#pragma once

// 海康 (Hikrobot MVS) 相机后端：MV-CS060-10UC-PRO + 类似型号。
// 仅在 CMake 检测到 /opt/MVS 时编译（HAS_HIK_SDK）。
//
// 明天到货后填充 TODO 部分：参考 luckyluckydadada/HIKROBOT-MVS-CAMERA-ROS
// 主要 API:
//   MV_CC_Initialize / MV_CC_Finalize
//   MV_CC_EnumDevices, MV_CC_CreateHandle, MV_CC_OpenDevice
//   MV_CC_SetEnumValue("PixelFormat" / "TriggerMode" / ...)
//   MV_CC_SetFloatValue("ExposureTime" / "Gain" / "Gamma")
//   MV_CC_SetIntValue("Width" / "Height")
//   MV_CC_StartGrabbing / MV_CC_GetImageBuffer / MV_CC_FreeImageBuffer

#include "camera_backend.h"

#ifdef HAS_HIK_SDK
  #include <MvCameraControl.h>
#endif

namespace tdt_vision {

class HikBackend : public CameraBackend {
public:
    HikBackend() = default;
    ~HikBackend() override;

    bool open(const CameraConfig& cfg, rclcpp::Logger logger) override;
    bool grab(cv::Mat& out_bgr, int timeout_ms = 200) override;
    void close() override;
    const char* brand() const override { return "hikvision"; }

#ifdef HAS_HIK_SDK
private:
    bool configure_device(const CameraConfig& cfg);
    bool convert_to_bgr(const MV_FRAME_OUT& frame, cv::Mat& out_bgr);

    void* handle_ = nullptr;          // MV_CC_HANDLE
    MV_FRAME_OUT frame_out_{};
    bool opened_ = false;

    rclcpp::Logger logger_ = rclcpp::get_logger("hikvision_backend");
#endif
};

}  // namespace tdt_vision
