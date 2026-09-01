// 海康 MVS 相机后端实现。
//
// 编译条件: 由 CMake 在检测到 SDK（项目内 MV_SDK/x86_64 或 /opt/MVS）后定义 HAS_HIK_SDK。
//   - 没装 SDK 时此 cpp 仍能编（提供占位实现），运行时 open() 返回 false。
//   - 装好 SDK 后会启用真实 MVS API 调用。
//
// 关键参考：
//   /opt/MVS/Samples/64/Linux64/  (官方 sample)
//   github.com/luckyluckydadada/HIKROBOT-MVS-CAMERA-ROS
//
// 适配相机：MV-CS060-10UC-PRO (USB3, 6MP, IMX178, Bayer)

#include "hik.h"

#include <cstdlib>
#include <cstring>
#include <sys/stat.h>

namespace tdt_vision {

#ifdef HAS_HIK_SDK

static void ensure_genicam_cache_dir()
{
    const char* dir = std::getenv("ALLUSERSPROFILE");
    if (dir && dir[0]) {
        mkdir(dir, 0755);
    }
}

HikBackend::~HikBackend() { HikBackend::close(); }

bool HikBackend::open(const CameraConfig& cfg, rclcpp::Logger logger)
{
    logger_ = logger;
    close();
    ensure_genicam_cache_dir();

    // 1. 枚举设备
    MV_CC_DEVICE_INFO_LIST device_list;
    std::memset(&device_list, 0, sizeof(device_list));
    int rc = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    if (rc != MV_OK) {
        RCLCPP_ERROR(logger_, "[hik] MV_CC_EnumDevices failed: 0x%x", rc);
        return false;
    }
    if (device_list.nDeviceNum == 0) {
        RCLCPP_WARN(logger_, "[hik] No USB camera detected — is it plugged in?");
        return false;
    }

    for (unsigned i = 0; i < device_list.nDeviceNum; ++i) {
        auto* info = device_list.pDeviceInfo[i];
        if (info && info->nTLayerType == MV_USB_DEVICE) {
            auto& usb = info->SpecialInfo.stUsb3VInfo;
            RCLCPP_INFO(logger_, "[hik] USB device[%u]: %s  (SN: %s)",
                        i, usb.chModelName, usb.chSerialNumber);
        }
    }

    int idx = -1;
    if (cfg.device_index <= 0) {
        for (unsigned i = 0; i < device_list.nDeviceNum; ++i) {
            auto* info = device_list.pDeviceInfo[i];
            if (info && info->nTLayerType == MV_USB_DEVICE) {
                const char* model = reinterpret_cast<const char*>(
                    info->SpecialInfo.stUsb3VInfo.chModelName);
                if (std::strstr(model, "MV-") != nullptr) {
                    idx = static_cast<int>(i);
                    RCLCPP_INFO(logger_, "[hik] Auto-selected device[%d]: %s", idx, model);
                    break;
                }
            }
        }
        if (idx < 0) {
            RCLCPP_WARN(logger_,
                "[hik] No device with 'MV-' in model name found among %u devices, "
                "falling back to device[0]", device_list.nDeviceNum);
            idx = 0;
        }
    } else {
        idx = cfg.device_index - 1;
        if (idx >= static_cast<int>(device_list.nDeviceNum)) {
            RCLCPP_WARN(logger_, "[hik] device_index=%d out of range (have %u), using 0",
                        cfg.device_index, device_list.nDeviceNum);
            idx = 0;
        }
    }
    RCLCPP_INFO(logger_, "[hik] Opening device[%d] ...", idx);

    // 2. 创建句柄并打开
    rc = MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[idx]);
    if (rc != MV_OK || handle_ == nullptr) {
        RCLCPP_ERROR(logger_, "[hik] CreateHandle failed: 0x%x", rc);
        return false;
    }
    rc = MV_CC_OpenDevice(handle_);
    if (rc != MV_OK) {
        const char* hint = "";
        if (rc == 0x80000203) hint = " (ACCESS_DENIED: check USB permissions / udev rules)";
        else if (rc == 0x80000204) hint = " (BUSY: camera may be held by another process)";
        RCLCPP_ERROR(logger_, "[hik] OpenDevice failed: 0x%x%s", rc, hint);
        MV_CC_DestroyHandle(handle_);
        handle_ = nullptr;
        return false;
    }

    if (!configure_device(cfg)) {
        close();
        return false;
    }

    rc = MV_CC_StartGrabbing(handle_);
    if (rc != MV_OK) {
        RCLCPP_ERROR(logger_, "[hik] StartGrabbing failed: 0x%x", rc);
        close();
        return false;
    }

    opened_ = true;
    return true;
}

void HikBackend::close()
{
    if (handle_ != nullptr) {
        MV_CC_StopGrabbing(handle_);
        MV_CC_CloseDevice(handle_);
        MV_CC_DestroyHandle(handle_);
        handle_ = nullptr;
    }
    opened_ = false;
}

bool HikBackend::configure_device(const CameraConfig& cfg)
{
    // 关闭触发模式（连续采集）
    int rc = MV_CC_SetEnumValue(handle_, "TriggerMode", 0);  // 0=Off
    if (rc != MV_OK) {
        RCLCPP_WARN(logger_, "[hik] TriggerMode=Off failed: 0x%x", rc);
    }

    // 曝光：> 0 手动，否则自动
    if (cfg.exposure_time > 0) {
        MV_CC_SetEnumValue(handle_, "ExposureAuto", 0);  // 0=Off
        rc = MV_CC_SetFloatValue(handle_, "ExposureTime",
                                 static_cast<float>(cfg.exposure_time));
        if (rc != MV_OK) {
            RCLCPP_WARN(logger_, "[hik] ExposureTime set failed: 0x%x", rc);
        }
    } else {
        MV_CC_SetEnumValue(handle_, "ExposureAuto", 2);  // 2=Continuous
    }

    // 增益
    if (cfg.gain > 0.1) {
        MV_CC_SetEnumValue(handle_, "GainAuto", 0);
        rc = MV_CC_SetFloatValue(handle_, "Gain", static_cast<float>(cfg.gain));
        if (rc != MV_OK) {
            RCLCPP_WARN(logger_, "[hik] Gain set failed: 0x%x", rc);
        }
    } else {
        MV_CC_SetEnumValue(handle_, "GainAuto", 2);
    }

    // Gamma
    if (cfg.gamma > 0.1 && std::abs(cfg.gamma - 1.0) > 0.01) {
        MV_CC_SetBoolValue(handle_, "GammaEnable", true);
        MV_CC_SetEnumValue(handle_, "GammaSelector", 1);  // 1=User
        rc = MV_CC_SetFloatValue(handle_, "Gamma", static_cast<float>(cfg.gamma));
        if (rc != MV_OK) {
            RCLCPP_WARN(logger_, "[hik] Gamma set failed: 0x%x", rc);
        }
    }

    // 白平衡
    if (cfg.balance_white_auto) {
        MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto", 1);  // 1=Continuous
    } else {
        MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto", 0);
        // TODO: 如有需要，按 BalanceRatioSelector + BalanceRatio 设置 R/G/B
    }

    // 分辨率：0 = 用相机硬件最大值（主动读取 WidthMax/HeightMax 并设置）
    {
        MVCC_INTVALUE_EX wmax{}, hmax{};
        int target_w = cfg.width;
        int target_h = cfg.height;
        if (target_w <= 0) {
            if (MV_CC_GetIntValueEx(handle_, "WidthMax", &wmax) == MV_OK)
                target_w = static_cast<int>(wmax.nCurValue);
        }
        if (target_h <= 0) {
            if (MV_CC_GetIntValueEx(handle_, "HeightMax", &hmax) == MV_OK)
                target_h = static_cast<int>(hmax.nCurValue);
        }
        if (target_w > 0) MV_CC_SetIntValue(handle_, "Width",  target_w);
        if (target_h > 0) MV_CC_SetIntValue(handle_, "Height", target_h);
        RCLCPP_INFO(logger_, "[hik] Resolution set to %dx%d", target_w, target_h);
    }

    // 帧率
    if (cfg.fps > 0) {
        MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true);
        rc = MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate",
                                 static_cast<float>(cfg.fps));
        if (rc != MV_OK) {
            RCLCPP_WARN(logger_, "[hik] FrameRate set failed: 0x%x", rc);
        }
    }

    // 像素格式: 优先尝试 BGR8 / RGB8 / Bayer
    // PixelFormat enum 值参考 MV_PIXEL_FORMAT (0x02180014=BGR8, 0x02180015=RGB8 等)
    // 简化：先尝试设置成 BayerRG8（IMX178 默认就是 BayerRG），上层做 demosaic
    // TODO 明天对照 MvCameraControl.h 里的常量验证
    rc = MV_CC_SetEnumValueByString(handle_, "PixelFormat", "BayerRG8");
    if (rc != MV_OK) {
        RCLCPP_WARN(logger_, "[hik] PixelFormat=BayerRG8 failed: 0x%x", rc);
    }

    return true;
}

bool HikBackend::grab(cv::Mat& out_bgr, int timeout_ms)
{
    if (!opened_) return false;

    std::memset(&frame_out_, 0, sizeof(frame_out_));
    int rc = MV_CC_GetImageBuffer(handle_, &frame_out_, timeout_ms);
    if (rc != MV_OK) {
        return false;
    }

    bool ok = convert_to_bgr(frame_out_, out_bgr);
    MV_CC_FreeImageBuffer(handle_, &frame_out_);
    return ok;
}

bool HikBackend::convert_to_bgr(const MV_FRAME_OUT& frame, cv::Mat& out_bgr)
{
    int w = frame.stFrameInfo.nWidth;
    int h = frame.stFrameInfo.nHeight;
    if (w <= 0 || h <= 0 || frame.pBufAddr == nullptr) return false;

    auto fmt = frame.stFrameInfo.enPixelType;

    // 常见格式（值见 MvCameraControl.h）：
    //   PixelType_Gvsp_BGR8_Packed   -> 0x02180015
    //   PixelType_Gvsp_RGB8_Packed   -> 0x02180014
    //   PixelType_Gvsp_BayerRG8      -> 0x01080009
    //   PixelType_Gvsp_BayerGR8      -> 0x01080008
    //   PixelType_Gvsp_BayerGB8      -> 0x0108000A
    //   PixelType_Gvsp_BayerBG8      -> 0x0108000B
    //   PixelType_Gvsp_Mono8         -> 0x01080001

    if (fmt == PixelType_Gvsp_BGR8_Packed) {
        cv::Mat tmp(h, w, CV_8UC3, frame.pBufAddr);
        tmp.copyTo(out_bgr);
        return true;
    }
    if (fmt == PixelType_Gvsp_RGB8_Packed) {
        cv::Mat tmp(h, w, CV_8UC3, frame.pBufAddr);
        cv::cvtColor(tmp, out_bgr, cv::COLOR_RGB2BGR);
        return true;
    }
    if (fmt == PixelType_Gvsp_Mono8) {
        cv::Mat mono(h, w, CV_8UC1, frame.pBufAddr);
        cv::cvtColor(mono, out_bgr, cv::COLOR_GRAY2BGR);
        return true;
    }
    if (fmt == PixelType_Gvsp_BayerRG8 || fmt == PixelType_Gvsp_BayerGR8 ||
        fmt == PixelType_Gvsp_BayerGB8 || fmt == PixelType_Gvsp_BayerBG8) {
        cv::Mat raw(h, w, CV_8UC1, frame.pBufAddr);
        int code = cv::COLOR_BayerBG2BGR;
        if (fmt == PixelType_Gvsp_BayerGR8) code = cv::COLOR_BayerGB2BGR;
        else if (fmt == PixelType_Gvsp_BayerGB8) code = cv::COLOR_BayerGR2BGR;
        else if (fmt == PixelType_Gvsp_BayerBG8) code = cv::COLOR_BayerRG2BGR;
        cv::cvtColor(raw, out_bgr, code);
        return true;
    }

    RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 2000,
                         "[hik] Unsupported PixelFormat: 0x%x", static_cast<unsigned>(fmt));
    return false;
}

#else  // !HAS_HIK_SDK：占位实现，运行时报错

HikBackend::~HikBackend() = default;

bool HikBackend::open(const CameraConfig& /*cfg*/, rclcpp::Logger logger)
{
    RCLCPP_ERROR(logger,
        "[hik] Hikvision SDK not available at compile time. "
        "Install MVS to /opt/MVS and rebuild.");
    return false;
}

bool HikBackend::grab(cv::Mat& /*out*/, int /*timeout_ms*/) { return false; }
void HikBackend::close() {}

#endif

}  // namespace tdt_vision
