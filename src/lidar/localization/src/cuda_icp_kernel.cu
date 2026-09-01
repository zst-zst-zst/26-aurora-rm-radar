#include "cuda_icp_kernel.h"
#include <cstdio>

// ── CUDA kernel: brute-force nearest-neighbor search ─────────────────
__global__ void find_nearest_kernel(
    const float* __restrict__ source,  // [n_source × 3]
    int   n_source,
    const float* __restrict__ target,  // [n_target × 3]
    int   n_target,
    float* __restrict__ out_dist,      // [n_source] squared distances
    int*   __restrict__ out_idx,       // [n_source] target point indices
    float max_dist_sq)
{
    const int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= n_source) return;

    const float sx = source[idx * 3];
    const float sy = source[idx * 3 + 1];
    const float sz = source[idx * 3 + 2];

    float best_dist = max_dist_sq;
    int   best_idx  = -1;

    for (int j = 0; j < n_target; ++j) {
        const float dx = sx - target[j * 3];
        const float dy = sy - target[j * 3 + 1];
        const float dz = sz - target[j * 3 + 2];
        const float d  = dx * dx + dy * dy + dz * dz;
        if (d < best_dist) {
            best_dist = d;
            best_idx  = j;
        }
    }

    out_dist[idx] = best_dist;
    out_idx[idx]  = best_idx;
}

void launch_find_nearest(
    const float* d_source, int n_source,
    const float* d_target, int n_target,
    float* d_out_dist, int* d_out_idx,
    float max_dist_sq)
{
    const int threads = 256;
    const int blocks  = (n_source + threads - 1) / threads;
    find_nearest_kernel<<<blocks, threads>>>(
        d_source, n_source, d_target, n_target,
        d_out_dist, d_out_idx, max_dist_sq);
}

const char* cuda_get_last_error()
{
    cudaError_t err = cudaGetLastError();
    if (err == cudaSuccess) return "";
    static char buf[256];
    snprintf(buf, sizeof(buf), "CUDA error %d: %s", (int)err, cudaGetErrorString(err));
    return buf;
}
