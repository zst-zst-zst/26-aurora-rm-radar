#include "sr_infer.h"

#include <cuda_runtime.h>
#include <opencv2/imgproc.hpp>
#include <rclcpp/logging.hpp>
#include "NvidiaInterface.hpp"

namespace tdt_radar {

// FSRCNN x4 wrapper:
//   Input  NHWC [1, H,   W,   1]  float32 Y-channel (0-255 range, TF OpenCV dnn_superres convention)
//   Output NCHW [1, 1, 4H, 4W]    float32 Y-channel (0-255 range)
// For color images: convert BGR->YCrCb, SR the Y channel, bicubic-upscale Cr/Cb,
// merge back and convert to BGR.
class SRInferImpl : public SRInfer {
public:
    std::shared_ptr<trt::Infer> trt_;
    int                         max_h_ = 128, max_w_ = 128;
    int                         scale_ = 4;
    std::vector<float>          host_in_, host_out_;
    float*                      dev_in_ = nullptr;
    float*                      dev_out_ = nullptr;

    ~SRInferImpl() override {
        if (dev_in_)  cudaFree(dev_in_);
        if (dev_out_) cudaFree(dev_out_);
    }

    bool init(const std::string& engine_file, int max_h, int max_w) {
        trt_ = trt::load(engine_file);
        if (!trt_) return false;
        max_h_ = max_h;
        max_w_ = max_w;
        size_t in_numel  = 1 * max_h_ * max_w_ * 1;
        size_t out_numel = 1 * 1 * max_h_ * scale_ * max_w_ * scale_;
        cudaMalloc(&dev_in_,  in_numel  * sizeof(float));
        cudaMalloc(&dev_out_, out_numel * sizeof(float));
        host_in_.resize(in_numel);
        host_out_.resize(out_numel);
        return true;
    }

    cv::Mat forward(const cv::Mat& bgr) override {
        constexpr int kMinDim = 32;  // TRT engine min profile
        if (bgr.empty() || bgr.rows > max_h_ || bgr.cols > max_w_ ||
            bgr.rows < kMinDim || bgr.cols < kMinDim ||
            bgr.channels() != 3) {
            return cv::Mat();
        }
        int H = bgr.rows, W = bgr.cols;

        // Convert BGR -> YCrCb, extract Y channel.
        cv::Mat ycrcb;
        cv::cvtColor(bgr, ycrcb, cv::COLOR_BGR2YCrCb);
        std::vector<cv::Mat> ycrcb_ch;
        cv::split(ycrcb, ycrcb_ch);  // ycrcb_ch[0]=Y, [1]=Cr, [2]=Cb (uint8)

        // Copy Y (uint8) to float host buffer (NHWC layout, values 0-255).
        size_t plane = static_cast<size_t>(H) * W;
        for (int y = 0; y < H; ++y) {
            const uchar* row = ycrcb_ch[0].ptr<uchar>(y);
            float*       dst = host_in_.data() + static_cast<size_t>(y) * W;
            for (int x = 0; x < W; ++x) dst[x] = static_cast<float>(row[x]);
        }

        cudaMemcpy(dev_in_, host_in_.data(), plane * sizeof(float),
                   cudaMemcpyHostToDevice);

        // Set dynamic input shape: NHWC [1, H, W, 1]
        if (!trt_->set_run_dims(0, {1, H, W, 1})) return cv::Mat();
        std::vector<void*> bindings{dev_in_, dev_out_};
        if (!trt_->forward(bindings, nullptr)) return cv::Mat();

        int H2 = H * scale_, W2 = W * scale_;
        size_t plane_out = static_cast<size_t>(H2) * W2;
        cudaMemcpy(host_out_.data(), dev_out_, plane_out * sizeof(float),
                   cudaMemcpyDeviceToHost);

        // Build upscaled Y (uint8).
        cv::Mat y_up(H2, W2, CV_8UC1);
        for (int y = 0; y < H2; ++y) {
            uchar* row = y_up.ptr<uchar>(y);
            const float* src = host_out_.data() + static_cast<size_t>(y) * W2;
            for (int x = 0; x < W2; ++x) {
                float v = src[x];
                if (v < 0) v = 0;
                if (v > 255) v = 255;
                row[x] = static_cast<uchar>(v);
            }
        }

        // Bicubic-upscale Cr and Cb, then merge with SR Y.
        cv::Mat cr_up, cb_up;
        cv::resize(ycrcb_ch[1], cr_up, cv::Size(W2, H2), 0, 0, cv::INTER_CUBIC);
        cv::resize(ycrcb_ch[2], cb_up, cv::Size(W2, H2), 0, 0, cv::INTER_CUBIC);
        std::vector<cv::Mat> merged{y_up, cr_up, cb_up};
        cv::Mat ycrcb_up, bgr_up;
        cv::merge(merged, ycrcb_up);
        cv::cvtColor(ycrcb_up, bgr_up, cv::COLOR_YCrCb2BGR);
        return bgr_up;
    }
};

std::shared_ptr<SRInfer> SRInfer::load(const std::string& engine_file,
                                       int max_h, int max_w) {
    auto impl = std::make_shared<SRInferImpl>();
    if (!impl->init(engine_file, max_h, max_w)) return nullptr;
    return impl;
}

}  // namespace tdt_radar
