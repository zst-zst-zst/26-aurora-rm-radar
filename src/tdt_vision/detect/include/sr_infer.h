#ifndef __SR_INFER_H__
#define __SR_INFER_H__

#include <memory>
#include <string>
#include <opencv2/core.hpp>

namespace tdt_radar {

// TensorRT-based ESPCN x4 super-resolution wrapper.
// Input : BGR cv::Mat of arbitrary HxW (within engine dynamic range)
// Output: BGR cv::Mat of (4H)x(4W)
class SRInfer {
public:
    static std::shared_ptr<SRInfer> load(const std::string& engine_file,
                                         int max_h = 128, int max_w = 128);
    virtual ~SRInfer() = default;
    // Returns empty Mat on failure or if input exceeds engine max shape.
    virtual cv::Mat forward(const cv::Mat& bgr) = 0;
};

}  // namespace tdt_radar

#endif
