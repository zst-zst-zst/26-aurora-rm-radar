// NodeCamera：HIK 海康相机节点（6MP USB）。

#include "node_camera.h"

#include "hik.h"

#include <cv_bridge/cv_bridge.hpp>

#include <algorithm>
#include <chrono>

namespace tdt_vision {

namespace {

// 把 brand 字符串规范化（小写）；支持别名
std::string normalize_brand(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    if (s == "hikvision" || s == "hikrobot" || s == "mvs") s = "hik";
    if (s == "automatic" || s == "") s = "hik";
    return s;
}

}  // namespace

NodeCamera::NodeCamera(const rclcpp::NodeOptions& options)
    : Node("vision_camera_node", options)
{
    camera_start_time_ = std::chrono::steady_clock::now();
    last_camera_diag_time_ = camera_start_time_;
    // ── 通用参数（不分品牌） ──
    brand_              = normalize_brand(this->declare_parameter<std::string>("brand", "hik"));
    frame_id_           = this->declare_parameter<std::string>("frame_id", "camera");
    image_topic_        = this->declare_parameter<std::string>("image_topic", "camera_image");
    compressed_topic_   = this->declare_parameter<std::string>("compressed_topic", "compressed_image");
    jpeg_quality_       = this->declare_parameter<int>("jpeg_quality", 85);
    publish_compressed_ = this->declare_parameter<bool>("publish_compressed", true);
    view_local_         = this->declare_parameter<bool>("view_local", false);

    // ── 各品牌独立参数声明（默认值已分别调好） ──
    // 海康 MV-CS060-10UC-PRO（IMX178, USB3, 6MP）
    this->declare_parameter<int>   ("hik.device_index",     1);
    this->declare_parameter<int>   ("hik.width",            0);   // 0 = 最大
    this->declare_parameter<int>   ("hik.height",           0);
    this->declare_parameter<int>   ("hik.fps",              30);
    this->declare_parameter<int>   ("hik.exposure_time",    5000);
    this->declare_parameter<double>("hik.gain",             10.0);
    this->declare_parameter<double>("hik.gamma",            1.6);
    this->declare_parameter<bool>  ("hik.balance_white_auto", true);
    this->declare_parameter<double>("hik.balance_ratio_red",   1.0);
    this->declare_parameter<double>("hik.balance_ratio_green", 1.0);
    this->declare_parameter<double>("hik.balance_ratio_blue",  1.0);

    // ── Publishers ──
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(image_topic_, qos);
    if (publish_compressed_) {
        compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
            compressed_topic_, qos);
    }

    // ── 打开初始 backend ──
    if (!switch_brand(brand_)) {
        RCLCPP_ERROR(this->get_logger(),
                     "Initial brand=%s failed to open. Node will keep retrying via param updates.",
                     brand_.c_str());
        RCLCPP_ERROR(this->get_logger(),
                     "【赛场体检-相机】相机打开失败：brand=%s。10秒内必须处理：检查海康USB3线/供电/MVS SDK/权限/是否被MVS客户端占用/设备序号。",
                     brand_.c_str());
        // 不 shutdown：保留节点，用户可改 brand 参数后重试
    }

    // ── 注册参数动态更新回调（运行时切换） ──
    param_cb_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& ps) {
            return this->on_set_param(ps);
        });

    // ── 启动采集线程 ──
    running_ = true;
    capture_thread_ = std::thread([this]() { grab_loop(); });
}

NodeCamera::~NodeCamera()
{
    running_ = false;
    if (capture_thread_.joinable()) capture_thread_.join();
    std::lock_guard<std::mutex> lk(backend_mu_);
    if (backend_) backend_->close();
}

CameraConfig NodeCamera::load_brand_cfg(const std::string& brand) const
{
    CameraConfig cfg;
    cfg.device_index   = this->get_parameter(brand + ".device_index").as_int();
    cfg.width          = this->get_parameter(brand + ".width").as_int();
    cfg.height         = this->get_parameter(brand + ".height").as_int();
    cfg.fps            = this->get_parameter(brand + ".fps").as_int();
    cfg.exposure_time  = this->get_parameter(brand + ".exposure_time").as_int();
    cfg.gain           = this->get_parameter(brand + ".gain").as_double();
    cfg.gamma          = this->get_parameter(brand + ".gamma").as_double();
    cfg.balance_white_auto = this->get_parameter(brand + ".balance_white_auto").as_bool();
    cfg.balance_ratio_red   = this->get_parameter(brand + ".balance_ratio_red").as_double();
    cfg.balance_ratio_green = this->get_parameter(brand + ".balance_ratio_green").as_double();
    cfg.balance_ratio_blue  = this->get_parameter(brand + ".balance_ratio_blue").as_double();
    cfg.encoding = "bgr8";
    return cfg;
}

std::unique_ptr<CameraBackend> NodeCamera::make_backend(const std::string& brand) const
{
    if (brand == "hik") return std::make_unique<HikBackend>();
    return nullptr;
}

bool NodeCamera::switch_brand(const std::string& new_brand)
{
    auto b = normalize_brand(new_brand);
    if (b != "hik") {
        RCLCPP_ERROR(this->get_logger(),
                     "Unknown brand='%s' (only hik is supported)", new_brand.c_str());
        RCLCPP_ERROR(this->get_logger(),
                     "【赛场体检-相机】相机品牌参数错误：brand=%s，目前只支持 hik。",
                     new_brand.c_str());
        return false;
    }

    std::lock_guard<std::mutex> lk(backend_mu_);
    if (backend_) {
        RCLCPP_INFO(this->get_logger(), "Closing current backend (%s)", backend_->brand());
        backend_->close();
        backend_.reset();
    }

    auto bk = make_backend(b);
    if (!bk) return false;

    auto cfg = load_brand_cfg(b);
    if (!bk->open(cfg, this->get_logger())) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to open backend=%s (check SDK install / device).", b.c_str());
        RCLCPP_ERROR(this->get_logger(),
                     "【赛场体检-相机】海康相机后端打开失败：backend=%s device_index=%d target=%dx%d@%dfps。可能是没插相机、SDK缺失、权限不足、相机被占用、USB带宽/供电异常。",
                     b.c_str(), cfg.device_index, cfg.width, cfg.height, cfg.fps);
        return false;
    }
    backend_ = std::move(bk);
    brand_   = b;
    RCLCPP_INFO(this->get_logger(),
                "Camera backend active: %s, target %dx%d @ %dfps, exp=%dus, gain=%.1f",
                brand_.c_str(), cfg.width, cfg.height, cfg.fps, cfg.exposure_time, cfg.gain);
    RCLCPP_INFO(this->get_logger(),
                "【赛场体检-相机】相机已打开：backend=%s target=%dx%d@%dfps，等待第一帧图像。",
                brand_.c_str(), cfg.width, cfg.height, cfg.fps);
    return true;
}

rcl_interfaces::msg::SetParametersResult NodeCamera::on_set_param(
    const std::vector<rclcpp::Parameter>& params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    bool need_reopen = false;
    std::string new_brand = brand_;

    for (const auto& p : params) {
        const auto& name = p.get_name();
        if (name == "brand") {
            new_brand = normalize_brand(p.as_string());
            need_reopen = true;
        } else if (name.rfind(brand_ + ".", 0) == 0) {
            // 当前品牌的参数被改：重开
            need_reopen = true;
        }
        // 其他品牌的参数改了不影响当前
    }

    if (need_reopen) {
        // 延迟到 grab_loop 线程执行 reopen，避免阻塞参数服务
        {
            std::lock_guard<std::mutex> lk(backend_mu_);
            pending_brand_ = new_brand;
        }
        reopen_pending_ = true;
    }
    return result;
}

void NodeCamera::grab_loop()
{
    cv::Mat frame;
    while (rclcpp::ok() && running_) {
        // Handle deferred reopen from parameter callback
        if (reopen_pending_.exchange(false)) {
            std::string brand;
            {
                std::lock_guard<std::mutex> lk(backend_mu_);
                brand = pending_brand_;
            }
            switch_brand(brand);
        }

        std::shared_ptr<CameraBackend> bk;
        {
            std::lock_guard<std::mutex> lk(backend_mu_);
            bk = backend_;
        }
        if (!bk) {
            const auto now = std::chrono::steady_clock::now();
            if (now - last_camera_diag_time_ >= std::chrono::seconds(2)) {
                last_camera_diag_time_ = now;
                RCLCPP_ERROR(this->get_logger(),
                             "【赛场体检-相机】没有可用相机后端：相机没有打开成功，所以不会发布图像。请立即检查相机连接/SDK/权限。");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }

        if (!bk->grab(frame, 200)) {
            ++failed_grab_count_;
            ++consecutive_grab_failures_;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "grab failed (brand=%s)", bk->brand());
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                  "【赛场体检-相机】相机已打开但抓不到图像：brand=%s 连续失败=%d 总失败=%llu。结论：不是detect问题，是相机采集失败/USB带宽/曝光触发/相机掉线。",
                                  bk->brand(), consecutive_grab_failures_,
                                  static_cast<unsigned long long>(failed_grab_count_));
            continue;
        }
        if (frame.empty()) continue;
        consecutive_grab_failures_ = 0;
        ++published_frame_count_;
        const auto now = std::chrono::steady_clock::now();
        if (!first_frame_reported_) {
            first_frame_reported_ = true;
            RCLCPP_INFO(this->get_logger(),
                        "【赛场体检-相机】第一帧图像已发布：size=%dx%d topic=%s。说明相机打开和采集都正常。",
                        frame.cols, frame.rows, image_topic_.c_str());
        }
        if (now - last_camera_diag_time_ >= std::chrono::seconds(2)) {
            last_camera_diag_time_ = now;
            const double run_s =
                std::max(0.001, std::chrono::duration<double>(now - camera_start_time_).count());
            const double fps = static_cast<double>(published_frame_count_) / run_s;
            const char *level = fps < 15.0 ? "偏低" : "正常";
            RCLCPP_INFO(this->get_logger(),
                        "【赛场体检-相机】%s：发布FPS=%.1f 已发布=%llu 抓帧失败=%llu 当前分辨率=%dx%d。若FPS低，优先查USB3口/线材/曝光/相机fps参数/CPU拷贝压力。",
                        level, fps,
                        static_cast<unsigned long long>(published_frame_count_),
                        static_cast<unsigned long long>(failed_grab_count_),
                        frame.cols, frame.rows);
        }

        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = frame_id_;
        msg->height   = frame.rows;
        msg->width    = frame.cols;
        msg->encoding = "bgr8";
        msg->is_bigendian = false;
        msg->step  = frame.cols * frame.channels();
        msg->data.assign(frame.datastart, frame.dataend);

        if (publish_compressed_ && compressed_pub_ &&
            compressed_pub_->get_subscription_count() > 0) {
            std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
            std::vector<uchar> buf;
            if (cv::imencode(".jpg", frame, buf, params)) {
                sensor_msgs::msg::CompressedImage cmsg;
                cmsg.header = msg->header;
                cmsg.format = "bgr8; jpeg compressed bgr8";
                cmsg.data = std::move(buf);
                compressed_pub_->publish(cmsg);
            }
        }

        if (view_local_) {
            cv::imshow("Local Camera View", frame);
            cv::waitKey(1);
        }
        image_pub_->publish(std::move(msg));
    }
}

}  // namespace tdt_vision

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(tdt_vision::NodeCamera)
