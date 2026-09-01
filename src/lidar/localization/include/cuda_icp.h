#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

/**
 * CUDA-accelerated ICP scan-to-map matching.
 *
 * Brute-force nearest-neighbor search on GPU (fast for up to ~100K points),
 * SVD transform estimation on CPU.
 *
 * @param source      Input source cloud (lidar scan, rm_frame)
 * @param target      Target cloud (map, rm_frame)
 * @param transform   Initial guess / output refined transform (rm_frame→livox_frame)
 * @param max_dist    Maximum correspondence distance (m). Points farther apart are rejected.
 * @param max_iter    Maximum ICP iterations
 * @param epsilon     Convergence threshold on transform delta (translation m + rotation rad)
 * @return            true if converged, false if max_iter reached
 */
/**
 * @param out_rmse     If non-null, written with the RMSE of final inlier
 *                     correspondences (m). Lower = better alignment.
 *                     Typical: <0.05 excellent, 0.05-0.15 good, >0.3 poor.
 * @param out_n_match  If non-null, written with the number of inlier
 *                     correspondences in the final iteration.
 */
bool cuda_icp_align(
    const pcl::PointCloud<pcl::PointXYZ>& source,
    const pcl::PointCloud<pcl::PointXYZ>& target,
    Eigen::Matrix4f& transform,
    float max_dist = 1.0f,
    int    max_iter = 50,
    float  epsilon  = 0.001f,
    float* out_rmse = nullptr,
    int*   out_n_match = nullptr);
