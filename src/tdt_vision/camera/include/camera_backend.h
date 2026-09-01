#pragma once

// 相机后端抽象接口：所有 SDK 实现都派生自此基类。
// 设计目标：
//   - node_camera.cpp 只依赖此头文件，不依赖具体 SDK
//   - 每个 backend 自管 SDK 句柄、内部参数、图像转 BGR
//   - 错误处理通过 open()/grab() 返回值传达

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace tdt_vision {

// 通用相机配置（从 ROS 参数读出来，传给 backend）
struct CameraConfig {
    int    device_index   = 1;
    std::string device_path;
    int    width          = 0;        // 0 = 用相机最大值
    int    height         = 0;
    int    fps            = 60;
    int    exposure_time  = 6000;     // us，<=0 = 自动
    double gain           = 12.0;     // <=0.1 = 自动
    double gamma          = 1.6;      // <=0.1 或 接近 1.0 = 不设置
    bool   balance_white_auto = true;
    double balance_ratio_red   = 1.0;
    double balance_ratio_green = 1.0;
    double balance_ratio_blue  = 1.0;
    bool   swap_rb        = false;    // 颜色通道翻转开关
    std::string encoding  = "bgr8";   // ROS sensor_msgs::Image 的 encoding 值
};

class CameraBackend {
public:
    virtual ~CameraBackend() = default;

    // 打开并初始化相机（包含 SDK 初始化、参数配置、采集启动）。
    // 返回 true 表示成功，可以开始 grab()。
    virtual bool open(const CameraConfig& cfg, rclcpp::Logger logger) = 0;

    // 阻塞式抓一帧，转换为 BGR cv::Mat 输出。
    // - timeout_ms: 等待超时
    // - 返回 true 且 out 非空表示成功
    virtual bool grab(cv::Mat& out_bgr, int timeout_ms = 200) = 0;

    // 停止采集并释放资源。可重复调用。
    virtual void close() = 0;

    // 返回品牌名，用于日志（"hikvision"）。
    virtual const char* brand() const = 0;
};

}  // namespace tdt_vision
