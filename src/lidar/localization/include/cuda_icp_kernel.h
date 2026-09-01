#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Launch CUDA brute-force nearest-neighbor search.
 * All pointers must be device pointers.
 */
void launch_find_nearest(
    const float* d_source, int n_source,
    const float* d_target, int n_target,
    float* d_out_dist, int* d_out_idx,
    float max_dist_sq);

/**
 * Get last CUDA error string (for logging). Returns empty string if no error.
 */
const char* cuda_get_last_error();

#ifdef __cplusplus
}
#endif
