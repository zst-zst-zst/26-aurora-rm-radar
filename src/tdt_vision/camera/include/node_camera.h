#pragma once

// HIK 海康相机节点。
//   - 启动时通过 ROS 参数 `brand` 选择
//   - 运行时可通过 SetParameters 切换 brand（节点内部重新打开）
//   - brand 固定为 hik，hik.* 参数控制曝光/增益等

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <sys/socket.h>
#include <netinet/in.h>

#include "camera_backend.h"

// Forward declare FFmpeg structures to avoid polluting namespace
struct AVCodec;
struct AVCodecContext;
struct AVFrame;
struct SwsContext;
struct AVPacket;

namespace tdt_vision {

class NodeCamera : public rclcpp::Node {
public:
    explicit NodeCamera(const rclcpp::NodeOptions& options);
    ~NodeCamera();

private:
    // 读 brand_ 对应的 ROS 参数命名空间，构造 CameraConfig
    CameraConfig load_brand_cfg(const std::string& brand) const;
    // 创建 backend 实例（不 open）
    std::unique_ptr<CameraBackend> make_backend(const std::string& brand) const;
    // 关掉旧 backend，打开新 brand。线程安全
    bool switch_brand(const std::string& new_brand);

    void grab_loop();
    rcl_interfaces::msg::SetParametersResult on_set_param(
        const std::vector<rclcpp::Parameter>& params);

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    std::mutex backend_mu_;
    std::shared_ptr<CameraBackend> backend_;
    std::string brand_;            // 当前生效品牌
    std::thread capture_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> reopen_pending_{false};
    std::string pending_brand_;              // protected by backend_mu_
    std::chrono::steady_clock::time_point last_reopen_attempt_{};
    int consecutive_grab_failures_{0};
    uint64_t published_frame_count_ = 0;
    uint64_t failed_grab_count_ = 0;
    bool first_frame_reported_ = false;
    std::chrono::steady_clock::time_point camera_start_time_{};
    std::chrono::steady_clock::time_point last_camera_diag_time_{};

    // 节点级（与品牌无关）
    std::string frame_id_;
    std::string image_topic_;
    std::string compressed_topic_;
    int  jpeg_quality_;
    bool publish_compressed_;
    bool view_local_;
};

}  // namespace tdt_vision
