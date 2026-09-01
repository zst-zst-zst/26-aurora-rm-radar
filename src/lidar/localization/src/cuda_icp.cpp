#include "cuda_icp.h"
#include "cuda_icp_kernel.h"
#include <cuda_runtime.h>
#include <iostream>
#include <vector>
#include <chrono>
#include <cmath>

// ── Apply transform to point array (in-place safe) ──────────────────
static void apply_transform(
    float* pts, int N, const Eigen::Matrix4f& T)
{
    const float r00 = T(0,0), r01 = T(0,1), r02 = T(0,2), tx = T(0,3);
    const float r10 = T(1,0), r11 = T(1,1), r12 = T(1,2), ty = T(1,3);
    const float r20 = T(2,0), r21 = T(2,1), r22 = T(2,2), tz = T(2,3);
    for (int i = 0; i < N; ++i) {
        const float x = pts[i*3], y = pts[i*3+1], z = pts[i*3+2];
        pts[i*3]   = r00*x + r01*y + r02*z + tx;
        pts[i*3+1] = r10*x + r11*y + r12*z + ty;
        pts[i*3+2] = r20*x + r21*y + r22*z + tz;
    }
}

// ── Public API ──────────────────────────────────────────────────────
bool cuda_icp_align(
    const pcl::PointCloud<pcl::PointXYZ>& source,
    const pcl::PointCloud<pcl::PointXYZ>& target,
    Eigen::Matrix4f& transform,
    float max_dist,
    int   max_iter,
    float epsilon,
    float* out_rmse,
    int*   out_n_match)
{
    if (out_rmse)    *out_rmse = -1.0f;
    if (out_n_match) *out_n_match = 0;
    const int Ns = static_cast<int>(source.size());
    const int Nt = static_cast<int>(target.size());
    if (Ns == 0 || Nt == 0) {
        std::cerr << "[cuda_icp] empty cloud(s): source=" << Ns << " target=" << Nt << std::endl;
        return false;
    }

    auto t_start = std::chrono::steady_clock::now();

    // ── Allocate device memory ───────────────────────────────────────
    float *d_src = nullptr, *d_tgt = nullptr, *d_dst = nullptr;
    int   *d_idx = nullptr;
    float *h_src = nullptr;
    float *h_dist = nullptr;
    int   *h_idx  = nullptr;

    const size_t src_bytes = static_cast<size_t>(Ns) * 3 * sizeof(float);
    const size_t tgt_bytes = static_cast<size_t>(Nt) * 3 * sizeof(float);

    auto cu = [](cudaError_t e, const char* desc) {
        if (e != cudaSuccess) {
            std::cerr << "[cuda_icp] " << desc << ": "
                      << cudaGetErrorString(e) << std::endl;
            return false;
        }
        return true;
    };

    if (!cu(cudaMalloc(&d_src, src_bytes), "malloc src")) return false;
    if (!cu(cudaMalloc(&d_tgt, tgt_bytes), "malloc tgt")) return false;
    if (!cu(cudaMalloc(&d_dst, static_cast<size_t>(Ns) * sizeof(float)), "malloc dst")) return false;
    if (!cu(cudaMalloc(&d_idx, static_cast<size_t>(Ns) * sizeof(int)), "malloc idx")) return false;

    h_src  = new float[Ns * 3];
    h_dist = new float[Ns];
    h_idx  = new int[Ns];

    // Upload target (fixed)
    std::vector<float> h_tgt(Nt * 3);
    for (int i = 0; i < Nt; ++i) {
        h_tgt[i*3]   = target[i].x;
        h_tgt[i*3+1] = target[i].y;
        h_tgt[i*3+2] = target[i].z;
    }
    cudaMemcpy(d_tgt, h_tgt.data(), tgt_bytes, cudaMemcpyHostToDevice);

    // Initial source — copy raw points then PRE-TRANSFORM by the caller-provided
    // initial guess. Without this, ICP starts from identity and ignores the PnP
    // initial pose entirely, causing the iterative SVD to drift to spurious
    // local minima (observed: transform diverging to 100+ m off-field).
    // Subsequent T_delta updates accumulate onto `transform` so the returned
    // matrix is the correct full livox→target mapping.
    for (int i = 0; i < Ns; ++i) {
        h_src[i*3]   = source[i].x;
        h_src[i*3+1] = source[i].y;
        h_src[i*3+2] = source[i].z;
    }
    apply_transform(h_src, Ns, transform);

    const float max_dist_sq = max_dist * max_dist;

    int  iter      = 0;
    bool converged = false;
    float prev_delta = 1e10f;
    int   osc_count  = 0;

    for (; iter < max_iter; ++iter) {
        // Upload current source
        cudaMemcpy(d_src, h_src, src_bytes, cudaMemcpyHostToDevice);

        // Launch nearest-neighbor kernel
        launch_find_nearest(d_src, Ns, d_tgt, Nt, d_dst, d_idx, max_dist_sq);
        const char* err = cuda_get_last_error();
        if (err[0] != '\0') {
            std::cerr << "[cuda_icp] kernel error: " << err << std::endl;
            break;
        }

        // Download correspondences
        cudaMemcpy(h_dist, d_dst, static_cast<size_t>(Ns) * sizeof(float), cudaMemcpyDeviceToHost);
        cudaMemcpy(h_idx,  d_idx, static_cast<size_t>(Ns) * sizeof(int),   cudaMemcpyDeviceToHost);

        // Build correspondence list
        std::vector<float> src_match, tgt_match;
        src_match.reserve(Ns * 3);
        tgt_match.reserve(Ns * 3);

        for (int i = 0; i < Ns; ++i) {
            if (h_idx[i] < 0) continue;
            src_match.push_back(h_src[i*3]);
            src_match.push_back(h_src[i*3+1]);
            src_match.push_back(h_src[i*3+2]);
            const int j = h_idx[i];
            tgt_match.push_back(target[j].x);
            tgt_match.push_back(target[j].y);
            tgt_match.push_back(target[j].z);
        }

        const int n_match = static_cast<int>(src_match.size()) / 3;
        if (n_match < 4) break;  // need at least 4 points for stable SVD

        // Compute centroids
        float cx_s = 0, cy_s = 0, cz_s = 0;
        float cx_t = 0, cy_t = 0, cz_t = 0;
        for (int i = 0; i < n_match; ++i) {
            cx_s += src_match[i*3];   cy_s += src_match[i*3+1];   cz_s += src_match[i*3+2];
            cx_t += tgt_match[i*3];   cy_t += tgt_match[i*3+1];   cz_t += tgt_match[i*3+2];
        }
        const float inv_n = 1.0f / n_match;
        cx_s *= inv_n; cy_s *= inv_n; cz_s *= inv_n;
        cx_t *= inv_n; cy_t *= inv_n; cz_t *= inv_n;

        // Cross-covariance matrix H (3×3)
        float H_data[9] = {0};
        for (int i = 0; i < n_match; ++i) {
            const float xs = src_match[i*3]   - cx_s;
            const float ys = src_match[i*3+1] - cy_s;
            const float zs = src_match[i*3+2] - cz_s;
            const float xt = tgt_match[i*3]   - cx_t;
            const float yt = tgt_match[i*3+1] - cy_t;
            const float zt = tgt_match[i*3+2] - cz_t;
            H_data[0] += xs * xt; H_data[1] += xs * yt; H_data[2] += xs * zt;
            H_data[3] += ys * xt; H_data[4] += ys * yt; H_data[5] += ys * zt;
            H_data[6] += zs * xt; H_data[7] += zs * yt; H_data[8] += zs * zt;
        }

        // SVD on CPU via Eigen
        Eigen::Map<Eigen::Matrix3f> H(H_data);
        Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3f R_delta = svd.matrixU() * svd.matrixV().transpose();
        if (R_delta.determinant() < 0) {
            Eigen::Matrix3f V = svd.matrixV();
            V.col(2) *= -1;
            R_delta = svd.matrixU() * V.transpose();
        }

        // Translation
        Eigen::Vector3f t_delta;
        t_delta.x() = cx_t - (R_delta(0,0)*cx_s + R_delta(0,1)*cy_s + R_delta(0,2)*cz_s);
        t_delta.y() = cy_t - (R_delta(1,0)*cx_s + R_delta(1,1)*cy_s + R_delta(1,2)*cz_s);
        t_delta.z() = cz_t - (R_delta(2,0)*cx_s + R_delta(2,1)*cy_s + R_delta(2,2)*cz_s);

        // Update cumulative transform: T = T_delta * T
        Eigen::Matrix4f T_delta = Eigen::Matrix4f::Identity();
        T_delta.block<3,3>(0,0) = R_delta;
        T_delta.block<3,1>(0,3) = t_delta;
        transform = T_delta * transform;

        // Apply transform to source for next iteration
        apply_transform(h_src, Ns, T_delta);

        // Convergence check
        const float delta = t_delta.norm() + (R_delta - Eigen::Matrix3f::Identity()).norm();
        if (delta < epsilon) { converged = true; break; }
        // Oscillation detector: require sustained divergence (3 consecutive
        // increases of >15%) before giving up.  Cold-start with large
        // max_corr_dist naturally has noisy early deltas; the previous 5%
        // single-frame trigger was too aggressive and caused premature exit.
        if (iter > 10 && delta > prev_delta * 1.15f) {
            ++osc_count;
            if (osc_count >= 3) break;
        } else {
            osc_count = 0;
        }
        prev_delta = delta;
    }

    // ── Final RMSE evaluation (one extra NN pass on the converged source) ──
    // h_src has been progressively transformed throughout the iterations and
    // now sits in target-frame coordinates. Re-run nearest-neighbor once to
    // measure how well source actually aligns to target (point-to-point).
    float rmse = -1.0f;
    int   n_inlier = 0;
    {
        cudaMemcpy(d_src, h_src, src_bytes, cudaMemcpyHostToDevice);
        launch_find_nearest(d_src, Ns, d_tgt, Nt, d_dst, d_idx, max_dist_sq);
        cudaMemcpy(h_dist, d_dst, static_cast<size_t>(Ns) * sizeof(float),
                   cudaMemcpyDeviceToHost);
        cudaMemcpy(h_idx, d_idx, static_cast<size_t>(Ns) * sizeof(int),
                   cudaMemcpyDeviceToHost);
        double sum_sq = 0.0;
        for (int i = 0; i < Ns; ++i) {
            if (h_idx[i] < 0) continue;
            sum_sq += h_dist[i];  // d_dst stores squared distance
            ++n_inlier;
        }
        if (n_inlier > 0) rmse = std::sqrt(static_cast<float>(sum_sq / n_inlier));
    }
    if (out_rmse)    *out_rmse    = rmse;
    if (out_n_match) *out_n_match = n_inlier;

    // Cleanup (after RMSE eval which reused these buffers)
    cudaFree(d_src);
    cudaFree(d_tgt);
    cudaFree(d_dst);
    cudaFree(d_idx);
    delete[] h_src;
    delete[] h_dist;
    delete[] h_idx;

    auto t_end = std::chrono::steady_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
    std::cout << "[cuda_icp] " << Ns << "→" << Nt << " pts, "
              << iter << " iters, " << ms << "ms, RMSE="
              << (rmse < 0 ? "n/a" : (std::to_string(rmse) + "m"))
              << ", inliers=" << n_inlier
              << (converged ? " CONVERGED" : " max_iter")
              << std::endl;

    return converged;
}
