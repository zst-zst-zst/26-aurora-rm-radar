#include "detect.h"

#include <filesystem>
#include <cmath>
#include <map>
#include <sstream>
#include <opencv2/imgproc.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>
#define TDT_INFO(msg) std::cout << msg << std::endl
#define MAX_CARS 12
#define MAX_ARMORS 20
namespace {

// Geometry of a (left, top, right, bottom) box pair: returns IoU and
// containment ratio (intersection / min_area).  Containment catches the
// "small box inside large box" case where IoU is artificially low because
// the union dominates — common with SAHI tile vs full-image overlaps.
struct BoxOverlap { float iou; float containment; };
template <class Box>
BoxOverlap boxOverlap(const Box& a, const Box& b) {
    const float ix1 = std::max(a.left, b.left);
    const float iy1 = std::max(a.top,  b.top);
    const float ix2 = std::min(a.right,  b.right);
    const float iy2 = std::min(a.bottom, b.bottom);
    const float inter  = std::max(0.f, ix2 - ix1) * std::max(0.f, iy2 - iy1);
    const float area_a = (a.right - a.left) * (a.bottom - a.top);
    const float area_b = (b.right - b.left) * (b.bottom - b.top);
    const float smaller = std::min(area_a, area_b);
    return { inter / (area_a + area_b - inter + 1e-6f),
             (smaller > 0.f) ? (inter / smaller) : 0.f };
}

// NMS with containment-aware + class-aware suppression.  Class label of -1
// means "ignore class" (single-class dedup).
yolo::BoxArray nmsMerge(yolo::BoxArray& boxes, float iou_thresh,
                        float containment_thresh = 0.70f) {
    std::sort(boxes.begin(), boxes.end(),
        [](const yolo::Box& a, const yolo::Box& b) { return a.confidence > b.confidence; });
    std::vector<bool> suppressed(boxes.size(), false);
    yolo::BoxArray result;
    for (size_t i = 0; i < boxes.size(); ++i) {
        if (suppressed[i]) continue;
        result.push_back(boxes[i]);
        for (size_t j = i + 1; j < boxes.size(); ++j) {
            if (suppressed[j]) continue;
            if (boxes[i].class_label != boxes[j].class_label) continue;
            const auto ov = boxOverlap(boxes[i], boxes[j]);
            if (ov.containment > containment_thresh || ov.iou > iou_thresh)
                suppressed[j] = true;
        }
    }
    return result;
}

std::string slotToDisplayText(int slot) {
    if (slot < 0 || slot >= tdt_radar::kRobotSlotCount) return "?";
    return std::to_string(tdt_radar::slot_to_robot_number(slot));
}

std::string armorClassToDisplayText(int class_label) {
    if (class_label == 0) return "?";
    const int slot = tdt_radar::armor_class_to_slot(class_label);
    if (slot < 0) return "X";
    return std::to_string(tdt_radar::slot_to_robot_number(slot));
}

}  // anonymous namespace

namespace tdt_radar {

int getColor(cv::Mat& img)
{
    if (img.empty()) return 1;
    const int h = img.rows, w = img.cols;
    // Zero out the central panel of a mask to suppress chassis/screen pixels;
    // we only want the red/blue light strips on the sides.
    auto zero_center = [&](cv::Mat& mask) {
        if (h > 4 && w > 4)
            mask(cv::Rect(w * 3 / 10, h * 2 / 10, w * 4 / 10, h * 6 / 10)).setTo(0);
    };

    // HSV color path: count saturated red/blue pixels (more robust).
    cv::Mat hsv;
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
    cv::Mat bright_mask;
    cv::inRange(hsv, cv::Scalar(0, 50, 80), cv::Scalar(180, 255, 255), bright_mask);
    zero_center(bright_mask);

    cv::Mat red1, red2, red_mask, blue_mask;
    cv::inRange(hsv, cv::Scalar(0,   70, 80), cv::Scalar(10,  255, 255), red1);
    cv::inRange(hsv, cv::Scalar(160, 70, 80), cv::Scalar(179, 255, 255), red2);
    cv::bitwise_or(red1, red2, red_mask);
    cv::inRange(hsv, cv::Scalar(90,  70, 80), cv::Scalar(130, 255, 255), blue_mask);
    cv::bitwise_and(red_mask,  bright_mask, red_mask);
    cv::bitwise_and(blue_mask, bright_mask, blue_mask);

    const int red_cnt   = cv::countNonZero(red_mask);
    const int blue_cnt  = cv::countNonZero(blue_mask);
    const int total_cnt = cv::countNonZero(bright_mask);
    if (total_cnt >= 4) {
        if (blue_cnt > red_cnt  * 12 / 10) return 0;
        if (red_cnt  > blue_cnt * 12 / 10) return 2;
    }

    // Fallback: bright-pixel mean BGR (HKUST light-strip method).
    cv::Mat gray, mask;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, mask, 160, 255, cv::THRESH_BINARY);
    zero_center(mask);
    if (cv::countNonZero(mask) < 2) return 1;
    const cv::Scalar mean_bgr = cv::mean(img, mask);
    if (mean_bgr[0] * 2 > mean_bgr[2] * 3) return 0;   // B > 1.5×R → blue
    if (mean_bgr[2] * 2 > mean_bgr[0] * 3) return 2;   // R > 1.5×B → red
    return 1;
}

cv::Rect getSafeRect(cv::Mat& image, cv::Rect& rect)
{
    cv::Rect save_rect;
    save_rect.x = std::max(0, rect.x);
    save_rect.y = std::max(0, rect.y);
    save_rect.width = std::min(image.cols - save_rect.x, rect.width);
    save_rect.height = std::min(image.rows - save_rect.y, rect.height);
    return save_rect;
}

Detect::Detect(const rclcpp::NodeOptions& node_options)
    : Node("radar_detect_node", node_options)
{
    detect_view_ = this->declare_parameter<bool>("detect_view", false);
    detect_view_scale_ =
        this->declare_parameter<double>("detect_view_scale", 1.0);
    force_draw_result_ =
        this->declare_parameter<bool>("force_draw_result", true);
    max_process_fps_ = this->declare_parameter<double>("max_process_fps", 0.0);
    drop_stale_ms_ = this->declare_parameter<int>("drop_stale_ms", 0);

    // ── 3D cuboid overlay (optional) ─────────────────────────────────────
    draw_3d_box_ = this->declare_parameter<bool>("draw_3d_box", true);
    const std::string box3d_camera_params = this->declare_parameter<std::string>(
        "camera_params", "src/tdt_vision/camera/config/hik.yaml");
    const std::string box3d_out_matrix = this->declare_parameter<std::string>(
        "out_matrix", "");
    box3d_map_height_ = this->declare_parameter<float>("map_height", 15.0F);
    box3d_referee_ = this->declare_parameter<bool>(
        "calibration_points_in_referee_frame", true);
    if (draw_3d_box_ && !box3d_out_matrix.empty()) {
        cv::FileStorage fs_cam(box3d_camera_params, cv::FileStorage::READ);
        cv::FileStorage fs_ext(box3d_out_matrix, cv::FileStorage::READ);
        if (fs_cam.isOpened() && fs_ext.isOpened()) {
            fs_cam["camera_matrix"] >> box3d_K_;
            fs_cam["dist_coeffs"]   >> box3d_D_;
            fs_ext["world_rvec"]    >> box3d_rvec_;
            fs_ext["world_tvec"]    >> box3d_tvec_;
            if (!box3d_K_.empty() && !box3d_rvec_.empty() && !box3d_tvec_.empty()) {
                cv::Rodrigues(box3d_rvec_, box3d_R_);
                box3d_Rt_ = box3d_R_.t();
                box3d_C_  = -box3d_Rt_ * box3d_tvec_;
                box3d_valid_ = true;
                RCLCPP_INFO(this->get_logger(),
                    "3D box overlay enabled: camera=%s, extrinsics=%s",
                    box3d_camera_params.c_str(), box3d_out_matrix.c_str());
            }
        }
        if (!box3d_valid_) {
            RCLCPP_WARN(this->get_logger(),
                "3D box overlay disabled: failed to load camera/extrinsics");
        }
    }
    std::cout << "Checking CUDA with nvidia-smi...\n";
    if (system("nvidia-smi > /dev/null 2>&1") == 0) {
        RCLCPP_INFO(this->get_logger(), "CUDA is available.");
    } else {
        RCLCPP_WARN(this->get_logger(), "nvidia-smi check failed (may be killed by signal). Continuing anyway...");
    }
    cv::FileStorage fs;
    fs.open("./config/detect_params.yaml", cv::FileStorage::READ);
    if (!fs.isOpened()) {
        RCLCPP_ERROR(this->get_logger(),
            "Failed to open ./config/detect_params.yaml — check working directory!");
        rclcpp::shutdown();
        return;
    }
    fs["yolo_path"] >> yolo_path;
    fs["yolo_onnx"] >> yolo_onnx;
    fs["armor_path"] >> armor_path;
    fs["classify_path"] >> classify_path;
    float yolo_conf = 0.45f, yolo_nms = 0.45f;
    float armor_conf = 0.35f, armor_nms = 0.45f;
    if (!fs["yolo_conf"].empty())  fs["yolo_conf"]  >> yolo_conf;
    if (!fs["yolo_nms"].empty())   fs["yolo_nms"]   >> yolo_nms;
    if (!fs["armor_conf"].empty()) fs["armor_conf"] >> armor_conf;
    if (!fs["armor_nms"].empty())  fs["armor_nms"]  >> armor_nms;
    if (!fs["sahi_enable"].empty()) { int v; fs["sahi_enable"] >> v; sahi_enable_ = (v != 0); }
    if (!fs["sahi_scale"].empty())     fs["sahi_scale"]     >> sahi_scale_;
    if (!fs["sahi_overlap"].empty())   fs["sahi_overlap"]   >> sahi_overlap_;
    if (!fs["sahi_merge_iou"].empty()) fs["sahi_merge_iou"] >> sahi_merge_iou_;
    if (!fs["sahi_y_band_top"].empty()) fs["sahi_y_band_top"] >> sahi_y_band_top_;
    if (!fs["sahi_y_band_bot"].empty()) fs["sahi_y_band_bot"] >> sahi_y_band_bot_;
    if (!fs["sr_enable"].empty()) { int v; fs["sr_enable"] >> v; sr_enable_ = (v != 0); }
    if (!fs["sr_min_dim"].empty())   fs["sr_min_dim"]   >> sr_min_dim_;
    if (!fs["sr_engine"].empty())    fs["sr_engine"]    >> sr_engine_;
    if (!fs["sr_max_dim"].empty())   fs["sr_max_dim"]   >> sr_max_dim_;
    fs.release();
    RCLCPP_INFO(this->get_logger(),
        "Detection thresholds: yolo_conf=%.2f yolo_nms=%.2f armor_conf=%.2f armor_nms=%.2f",
        yolo_conf, yolo_nms, armor_conf, armor_nms);
    RCLCPP_INFO(this->get_logger(),
        "SAHI: enable=%d scale=%.1f overlap=%.2f merge_iou=%.2f y_band=[%.2f, %.2f]",
        sahi_enable_, sahi_scale_, sahi_overlap_, sahi_merge_iou_,
        sahi_y_band_top_, sahi_y_band_bot_);
    RCLCPP_INFO(this->get_logger(),
        "SR: enable=%d min_dim=%d max_dim=%d engine=%s",
        sr_enable_, sr_min_dim_, sr_max_dim_, sr_engine_.c_str());
    if (sr_enable_ && !sr_engine_.empty()) {
        sr_infer_ = SRInfer::load(sr_engine_, sr_max_dim_, sr_max_dim_);
        if (!sr_infer_) {
            RCLCPP_WARN(this->get_logger(),
                "SR engine load failed, falling back to INTER_CUBIC");
        } else {
            RCLCPP_INFO(this->get_logger(), "SR engine loaded OK");
        }
    }

    if (yolo_onnx.empty()) {
        yolo_onnx = "model/ONNX/yolo.onnx";
    }

    std::ifstream file1(yolo_path.c_str());
    if (!file1.good()) {
        const std::string command = "python3 src/utils/onnx2trt.py "
                                    "--onnx=" + yolo_onnx + " "
                                    "--saveEngine=model/TensorRT/yolo.engine "
                                    "--minBatch 1 "
                                    "--optBatch 1 "
                                    "--maxBatch 2 "
                                    "--Shape=1280x1280 "
                                    "--input_name=images";
        system(command.c_str());
    } else {
        TDT_INFO("Load yolo engine!");
    }
    std::ifstream file2(armor_path.c_str());
    if (!file2.good()) {
        system("python3 src/utils/onnx2trt.py "
               "--onnx=model/ONNX/armor_yolo.onnx "
               "--saveEngine=model/TensorRT/armor_yolo.engine "
               "--minBatch 1 "
               "--optBatch 5 "
               "--maxBatch 12 "
               "--Shape=192x192 "
               "--input_name=images");
    } else {
        TDT_INFO("Load armor_yolo engine!");
    }
    std::ifstream file3(classify_path.c_str());
    if (!file3.good()) {
        system("python3 src/utils/onnx2trt.py "
               "--onnx=model/ONNX/classify.onnx "
               "--saveEngine=model/TensorRT/classify.engine "
               "--minBatch 1 "
               "--optBatch 10 "
               "--maxBatch 20 "
               "--Shape=224x224 "
               "--input_name=input");
    } else {
        TDT_INFO("Load classify engine!");
    }
    std::cout << "yolo_path:" << yolo_path << "\n";
    std::cout << "armor_path:" << armor_path << "\n";
    std::cout << "classify_path:" << classify_path << "\n";
    this->classifier =
        classify::load(classify_path, classify::Type::densenet121);
    TDT_INFO("Load classify engine success!");

    this->armor_yolo = yolo::load(armor_path, yolo::Type::V8, armor_conf, armor_nms);
    TDT_INFO("Load armor_yolo engine success!");
    this->yolo = yolo::load(yolo_path, yolo::Type::V8, yolo_conf, yolo_nms);
    TDT_INFO("Load yolo engine success!");
    image_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "camera_image", rclcpp::SensorDataQoS(),
        std::bind(&Detect::callback, this, std::placeholders::_1));
    pub = this->create_publisher<vision_interface::msg::DetectResult>(
        "detect_result", rclcpp::SensorDataQoS());
    debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "detect_debug_image", rclcpp::SensorDataQoS());
    RCLCPP_INFO(this->get_logger(), "Detect node has been started.");
}

void Detect::callback(const std::shared_ptr<sensor_msgs::msg::Image> msg)
{
    ++received_frame_count_;
    const int64_t msg_ts_ns = static_cast<int64_t>(msg->header.stamp.nanosec) +
                              static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL;
    const int64_t now_ts_ns = this->get_clock()->now().nanoseconds();
    const double now_sec = this->get_clock()->now().seconds();
    if (first_image_sec_ < 0.0) {
        first_image_sec_ = now_sec;
        RCLCPP_INFO(this->get_logger(),
            "【赛场体检-相机/识别】detect已收到第一帧图像：topic=camera_image size=%ux%u。说明相机图像链路已通。",
            msg->width, msg->height);
    }
    last_image_sec_ = now_sec;

    if (drop_stale_ms_ > 0 && msg_ts_ns > 0) {
        const int64_t stale_threshold_ns =
            static_cast<int64_t>(drop_stale_ms_) * 1000000LL;
        if ((now_ts_ns - msg_ts_ns) > stale_threshold_ns) {
            ++dropped_frame_count_;
            ++stale_drop_count_;
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "【赛场体检-识别】丢弃过期图像：age=%.1fms 阈值=%dms stale_drop=%llu total_drop=%llu。结论：detect处理不过来或系统/GPU太卡，正在处理旧帧。",
                (now_ts_ns - msg_ts_ns) / 1e6,
                drop_stale_ms_,
                static_cast<unsigned long long>(stale_drop_count_),
                static_cast<unsigned long long>(dropped_frame_count_));
            return;
        }
    }

    if (max_process_fps_ > 0.0) {
        const int64_t min_interval_ns = static_cast<int64_t>(
            std::max(1.0, 1e9 / max_process_fps_));
        const int64_t current_ts_ns = (msg_ts_ns > 0) ? msg_ts_ns : now_ts_ns;
        if (last_process_ts_ns_ > 0 &&
            (current_ts_ns - last_process_ts_ns_) < min_interval_ns) {
            ++dropped_frame_count_;
            ++throttle_drop_count_;
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "【赛场体检-识别】按最大处理帧率限流：max_process_fps=%.1f throttle_drop=%llu total_drop=%llu。",
                max_process_fps_,
                static_cast<unsigned long long>(throttle_drop_count_),
                static_cast<unsigned long long>(dropped_frame_count_));
            return;
        }
        last_process_ts_ns_ = current_ts_ns;
    }

    std::chrono::steady_clock::time_point begin =
        std::chrono::steady_clock::now();
    // toCvShare = zero-copy view onto the upstream message.  Inference (`yolo->forward`,
    // `tdt_radar::Image(img.data, ...)`) reads only.  Drawing for the debug view is
    // done on a *separate* clone inside `show_detect_view` to avoid corrupting the
    // shared message memory.
    auto             img = cv_bridge::toCvShare(msg, "bgr8")->image;
    vision_interface::msg::DetectResult detect_result;
    detect_result.header.stamp = msg->header.stamp;

    // Forward-declared so show_detect_view (defined below) can reference it.
    std::vector<Car> cars;

    // Draw overlays onto an independent copy (full-res first, then downscale).
    // No-op when detect_view_=false → zero CPU/memory cost in production.
    // ── 3D cuboid projector ────────────────────────────────────────────
    // Given bbox bottom-centre pixel, reverse-ray-cast to the ground plane,
    // shift away from the camera by HALF_CHASSIS to get the robot ground
    // centre, build an axis-aligned 0.5×0.5×0.5 m cuboid and project its 8
    // corners back into image space.  Returns false if the calibration is
    // unavailable or the ray points up.
    constexpr float kBoxHalfSide = 0.25f;   // half of 0.5 m chassis footprint
    constexpr float kBoxHeight   = 0.50f;   // typical infantry height
    constexpr float kHalfChassis = 0.26f;   // bbox bottom = near wheel, shift back
    auto compute_cuboid_pixels = [&](float px, float py,
                                      std::array<cv::Point2f, 8>& out) -> bool {
        if (!box3d_valid_) return false;
        // Undistort pixel → camera ray
        std::vector<cv::Point2f> undist;
        cv::undistortPoints(std::vector<cv::Point2f>{{px, py}}, undist,
                            box3d_K_, box3d_D_, cv::noArray(), box3d_K_);
        const double fx = box3d_K_.at<double>(0, 0);
        const double fy = box3d_K_.at<double>(1, 1);
        const double cx = box3d_K_.at<double>(0, 2);
        const double cy = box3d_K_.at<double>(1, 2);
        cv::Mat d_cam = (cv::Mat_<double>(3, 1)
            << (undist[0].x - cx) / fx, (undist[0].y - cy) / fy, 1.0);
        cv::Mat d_world = box3d_Rt_ * d_cam;
        const double cxw = box3d_C_.at<double>(0);
        const double cyw = box3d_C_.at<double>(1);
        const double czw = box3d_C_.at<double>(2);
        const double dz  = d_world.at<double>(2);
        if (std::abs(dz) < 1e-6 || (-czw / dz) < 0.0) return false;
        // Intersect ray with ground plane Z=0 (legacy world frame)
        const double t = -czw / dz;
        const double hit_x = cxw + t * d_world.at<double>(0);
        const double hit_y = cyw + t * d_world.at<double>(1);
        // Shift HALF_CHASSIS away from camera → robot ground centre
        const double vx = hit_x - cxw, vy = hit_y - cyw;
        const double vn = std::sqrt(vx*vx + vy*vy);
        if (vn < 0.5) return false;          // robot directly under camera
        const double rcx = hit_x + kHalfChassis * vx / vn;
        const double rcy = hit_y + kHalfChassis * vy / vn;
        // Build cuboid corners (legacy world frame, Z up)
        std::vector<cv::Point3f> corners = {
            {static_cast<float>(rcx - kBoxHalfSide), static_cast<float>(rcy - kBoxHalfSide), 0.0f},
            {static_cast<float>(rcx + kBoxHalfSide), static_cast<float>(rcy - kBoxHalfSide), 0.0f},
            {static_cast<float>(rcx + kBoxHalfSide), static_cast<float>(rcy + kBoxHalfSide), 0.0f},
            {static_cast<float>(rcx - kBoxHalfSide), static_cast<float>(rcy + kBoxHalfSide), 0.0f},
            {static_cast<float>(rcx - kBoxHalfSide), static_cast<float>(rcy - kBoxHalfSide), kBoxHeight},
            {static_cast<float>(rcx + kBoxHalfSide), static_cast<float>(rcy - kBoxHalfSide), kBoxHeight},
            {static_cast<float>(rcx + kBoxHalfSide), static_cast<float>(rcy + kBoxHalfSide), kBoxHeight},
            {static_cast<float>(rcx - kBoxHalfSide), static_cast<float>(rcy + kBoxHalfSide), kBoxHeight},
        };
        std::vector<cv::Point2f> pix;
        cv::projectPoints(corners, box3d_rvec_, box3d_tvec_,
                          box3d_K_, box3d_D_, pix);
        if (pix.size() != 8) return false;
        for (int k = 0; k < 8; ++k) out[k] = pix[k];
        return true;
    };
    auto draw_cuboid = [&](cv::Mat& img_ref,
                            const std::array<cv::Point2f, 8>& pix,
                            const cv::Scalar& color) {
        // 12 edges: bottom 0-1-2-3-0, top 4-5-6-7-4, verticals 0-4 1-5 2-6 3-7
        static constexpr int E[12][2] = {
            {0,1},{1,2},{2,3},{3,0},  // bottom
            {4,5},{5,6},{6,7},{7,4},  // top
            {0,4},{1,5},{2,6},{3,7},  // verticals
        };
        for (int i = 0; i < 12; ++i) {
            cv::line(img_ref, pix[E[i][0]], pix[E[i][1]], color,
                     (i < 4) ? 2 : 1, cv::LINE_AA);
        }
    };

    auto show_detect_view = [&](const std::string& hint) {
        if (!detect_view_) return;
        cv::Mat view_img = img.clone();  // independent buffer for drawing

        // Box / armor / id overlays (matches the legacy debug rendering).
        if (debug || force_draw_result_) {
            for (const auto& car : cars) {
                if (car.number < 0 || car.number >= kRobotSlotCount || car.color == 1) continue;
                if (car.is_held) continue;  // suppress ghost trails
                const cv::Rect label_rect = car.armor_identified ? car.armor_rect : car.car_rect;
                const cv::Scalar box_color   = (car.color == 0) ? cv::Scalar(255, 0, 0)   : cv::Scalar(0, 0, 255);
                const cv::Scalar armor_color = (car.color == 0) ? cv::Scalar(255,180,0)   : cv::Scalar(0,165,255);
                // 3D cuboid overlay (axis-aligned in world frame; appears as
                // a parallelepiped in perspective view).  Fall back to the 2D
                // bbox only when projection is unavailable.
                bool drew_cuboid = false;
                if (box3d_valid_) {
                    std::array<cv::Point2f, 8> pix;
                    if (compute_cuboid_pixels(car.center.x, car.center.y, pix)) {
                        draw_cuboid(view_img, pix, box_color);
                        drew_cuboid = true;
                    }
                }
                if (!drew_cuboid) {
                    cv::rectangle(view_img, car.car_rect, box_color, 2);
                }
                if (car.armor_identified)
                    cv::rectangle(view_img, car.armor_rect, armor_color, 2);
                if (car.has_detection_confidence) {
                    char buf[16];
                    std::snprintf(buf, sizeof(buf), "%.2f", car.detection_confidence);
                    cv::putText(view_img, buf,
                        cv::Point(car.car_rect.x, car.car_rect.y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255,255,255), 1);
                }
                cv::putText(view_img, std::string("ID:") + slotToDisplayText(car.number),
                    cv::Point(label_rect.x, std::max(25, label_rect.y - 8)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0,255,255), 2);
            }
        }
        // Per-armor classifier labels (debug mode only).
        if (debug) {
            for (const auto& car : cars) {
                for (const auto& armor : car.armors) {
                    cv::putText(view_img, armorClassToDisplayText(armor.class_label),
                        cv::Point(armor.left + car.car.left, armor.top + car.car.top),
                        cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,255,255), 2);
                }
            }
        }

        // Downscale for display.
        if (view_img.cols > 1536) {
            const double scale = 1536.0 / static_cast<double>(view_img.cols);
            cv::resize(view_img, view_img, cv::Size(), scale, scale, cv::INTER_LINEAR);
        }
        if (detect_view_scale_ > 1.01) {
            cv::resize(view_img, view_img, cv::Size(),
                       detect_view_scale_, detect_view_scale_, cv::INTER_LINEAR);
        }
        if (force_draw_result_) {
            cv::putText(view_img, hint, cv::Point(30, 60),
                        cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 255, 255), 2);
        }
        auto debug_msg = cv_bridge::CvImage(msg->header, "bgr8", view_img).toImageMsg();
        debug_image_pub_->publish(*debug_msg);
    };

    tdt_radar::Image image(img.data, img.cols, img.rows);

    auto result = yolo->forward(image);

    bool use_sahi = sahi_enable_;
    if (use_sahi && img.cols <= 1600 && img.rows <= 1200) {
        use_sahi = false;
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                              "SAHI auto-disabled for low-res input %dx%d",
                              img.cols, img.rows);
    }

    if (use_sahi) {
        const int tile_w = 1280, tile_h = 1280;
        const float inv_scale = 1.0f / sahi_scale_;
        cv::Mat upscaled;
        cv::resize(img, upscaled, cv::Size(), sahi_scale_, sahi_scale_, cv::INTER_LINEAR);
        const int uw = upscaled.cols, uh = upscaled.rows;
        const int step_x = static_cast<int>(tile_w * (1.0f - sahi_overlap_));
        const int step_y = static_cast<int>(tile_h * (1.0f - sahi_overlap_));

        yolo::BoxArray all_boxes = result;

        // SAHI only inside the configured image-y band.  The base full-image
        // YOLO pass above still covers the skipped ceiling / near-field areas.
        int strip_y0 = std::max(0, static_cast<int>(sahi_y_band_top_ * uh));
        int strip_y1 = std::min(uh, static_cast<int>(sahi_y_band_bot_ * uh));
        if (strip_y1 - strip_y0 < 64) { strip_y0 = 0; strip_y1 = uh; }

        for (int ty = strip_y0; ty < strip_y1; ty += step_y) {
            const int y2 = std::min(ty + tile_h, strip_y1);
            const int y1 = std::max(strip_y0, y2 - tile_h);
            for (int tx = 0; tx < uw; tx += step_x) {
                const int x2 = std::min(tx + tile_w, uw);
                const int x1 = std::max(0, x2 - tile_w);
                cv::Mat tile = upscaled(cv::Rect(x1, y1, x2 - x1, y2 - y1)).clone();
                tdt_radar::Image tile_img(tile.data, tile.cols, tile.rows);
                auto tile_result = yolo->forward(tile_img);
                for (auto& box : tile_result) {
                    box.left   = (box.left   + x1) * inv_scale;
                    box.top    = (box.top    + y1) * inv_scale;
                    box.right  = (box.right  + x1) * inv_scale;
                    box.bottom = (box.bottom + y1) * inv_scale;
                    all_boxes.push_back(box);
                }
            }
        }
        result = nmsMerge(all_boxes, sahi_merge_iou_);
    }

    // Filter by class FIRST, then check count — otherwise non-car
    // detections inflate the count and cause valid frames to be dropped.
    std::vector<tdt_radar::Image> images;
    std::vector<cv::Mat>          car_imgs;
    std::vector<float>            car_crop_scales;
    for (auto& box : result) {
        if (box.class_label == 0 || box.class_label == 1) {
            Car car;
            car.car = box;
            car.detection_confidence = box.confidence;
            car.has_detection_confidence = true;
            cars.push_back(car);
        }
    }

    // Containment-aware dedup: standard IoU misses cases where a small box
    // is inside a large box (IoU is low because the union is large).
    {
        std::sort(cars.begin(), cars.end(),
            [](const Car& a, const Car& b) {
                return a.detection_confidence > b.detection_confidence;
            });
        std::vector<bool> suppressed(cars.size(), false);
        for (size_t i = 0; i < cars.size(); ++i) {
            if (suppressed[i]) continue;
            for (size_t j = i + 1; j < cars.size(); ++j) {
                if (suppressed[j]) continue;
                if (boxOverlap(cars[i].car, cars[j].car).containment > 0.60f)
                    suppressed[j] = true;
            }
        }
        size_t w = 0;
        for (size_t i = 0; i < cars.size(); ++i)
            if (!suppressed[i]) cars[w++] = cars[i];
        cars.resize(w);
    }

    if (cars.empty()) {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                              "No Car! (yolo raw=%zu)", result.size());
        pub->publish(detect_result);
        show_detect_view("NO CAR");
        return;
    } else if (static_cast<int>(cars.size()) > MAX_CARS) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Too Many Cars: %zu (raw=%zu)",
                             cars.size(), result.size());
        cars.resize(MAX_CARS);  // keep the first MAX_CARS instead of dropping all
    }

    // Diagnostic: log class distribution periodically
    {
        static uint64_t diag_counter = 0;
        if ((++diag_counter % 200) == 1) {
            std::map<int, int> cls_count;
            for (const auto& box : result) cls_count[box.class_label]++;
            std::ostringstream oss;
            oss << "YOLO raw=" << result.size() << " cars=" << cars.size() << " cls:";
            for (auto& [k, v] : cls_count) oss << " " << k << "×" << v;
            RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
        }
    }

    std::vector<cv::Mat> car_imgs_clean;  // for classifier (no sharpening artifacts)
    for (auto& car : cars) {
        auto     temp_rect = cv::Rect(car.car.left, car.car.top,
                                      car.car.right - car.car.left,
                                      car.car.bottom - car.car.top);
        cv::Rect temp_car_rect = getSafeRect(img, temp_rect);
        cv::Mat  car_img = img(temp_car_rect).clone();
        float cs = 1.0f;
        int min_dim = std::min(car_img.cols, car_img.rows);
        if (min_dim > 0 && min_dim < sr_min_dim_) {
            // Try real SR (ESPCN x4) first if engine loaded and crop fits.
            cv::Mat sr_out;
            if (sr_infer_ &&
                car_img.rows <= sr_max_dim_ && car_img.cols <= sr_max_dim_) {
                sr_out = sr_infer_->forward(car_img);
            }
            if (!sr_out.empty()) {
                car_img = sr_out;
                cs = 4.0f;
                // Still under 192? top up with INTER_CUBIC.
                int md2 = std::min(car_img.cols, car_img.rows);
                if (md2 < 192) {
                    float extra = 192.0f / static_cast<float>(md2);
                    cv::resize(car_img, car_img, cv::Size(), extra, extra,
                               cv::INTER_CUBIC);
                    cs *= extra;
                }
            } else {
                cs = 192.0f / static_cast<float>(min_dim);
                cv::resize(car_img, car_img, cv::Size(), cs, cs,
                           cv::INTER_CUBIC);
            }
        }
        cv::Mat car_img_clean = (cs > 1.01f) ? car_img.clone() : car_img;
        // Sharpen only the version fed to armor_yolo (helps edge detection).
        if (cs > 1.01f) {
            cv::Mat blurred;
            cv::GaussianBlur(car_img, blurred, cv::Size(0, 0), 2.0);
            cv::addWeighted(car_img, 1.4, blurred, -0.4, 0.0, car_img);
        }
        car_crop_scales.push_back(cs);
        car_imgs.push_back(car_img);
        car_imgs_clean.push_back(car_img_clean);
        car.car_rect = temp_car_rect;
    }

    for (auto& car_img : car_imgs) {
        auto image =
            tdt_radar::Image(car_img.data, car_img.cols, car_img.rows);
        images.push_back(image);
    }

    auto armor_boxes = armor_yolo->forwards(images);
    bool has_armor = false;
    std::vector<std::vector<cv::Mat>> armor_hires_per_car(armor_boxes.size());
    for (size_t i = 0; i < armor_boxes.size(); i++) {
        if (armor_boxes[i].size() == 0) {
            continue;
        }
        for (auto& box : armor_boxes[i]) {
            cv::Rect r(static_cast<int>(box.left), static_cast<int>(box.top),
                       static_cast<int>(box.right - box.left),
                       static_cast<int>(box.bottom - box.top));
            // The digit classifier was trained on tight armor crops. Expanding this ROI
            // drags in large amounts of chassis/background and collapses many predictions
            // toward the same class (observed as every robot becoming ID 1).
            cv::Rect safe = getSafeRect(car_imgs_clean[i], r);
            if (safe.area() > 0) {
                armor_hires_per_car[i].push_back(car_imgs_clean[i](safe).clone());
            } else {
                armor_hires_per_car[i].push_back(cv::Mat());
            }
        }
        if (car_crop_scales[i] > 1.01f) {
            float inv = 1.0f / car_crop_scales[i];
            for (auto& box : armor_boxes[i]) {
                box.left   *= inv;
                box.top    *= inv;
                box.right  *= inv;
                box.bottom *= inv;
            }
        }
        cars[i].armors = armor_boxes[i];
        has_armor = true;
    }  // 将armor_boxes存储到cars中
    if (!has_armor) {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                              "No Armor on %zu cars, using fallback", cars.size());
    }
    std::vector<cv::Mat>         armor_imgs;
    std::vector<classify::Image> armor_images;

    for (size_t ci = 0; ci < cars.size(); ci++) {
        auto& car = cars[ci];
        if (car.armors.size() == 0) {
            continue;
        }
        for (size_t ai = 0; ai < car.armors.size(); ai++) {
            auto& armor = car.armors[ai];
            cv::Mat armor_img;
            if (ci < armor_hires_per_car.size() &&
                ai < armor_hires_per_car[ci].size() &&
                !armor_hires_per_car[ci][ai].empty()) {
                armor_img = armor_hires_per_car[ci][ai];
            } else {
                cv::Rect rect_img_1(
                    armor.left + car.car.left, armor.top + car.car.top,
                    armor.right - armor.left, armor.bottom - armor.top);
                cv::Rect rect_img = getSafeRect(img, rect_img_1);
                armor_img = img(rect_img).clone();
            }
            armor_imgs.push_back(armor_img);
        }
    }
    for (auto& armor_img : armor_imgs) {
        auto image =
            classify::Image(armor_img.data, armor_img.cols, armor_img.rows);
        armor_images.push_back(image);
    }
    std::vector<int> armor_result;
    if (!armor_images.empty()) {
        armor_result = classifier->forwards(armor_images);
        if (!armor_result.empty()) {
            std::ostringstream oss;
            oss << "armor id:";
            const int limit = std::min<int>(armor_result.size(), 8);
            for (int i = 0; i < limit; ++i) {
                oss << ' ' << armorClassToDisplayText(armor_result[i]);
            }
            if (static_cast<int>(armor_result.size()) > limit) oss << " ...";
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1500,
                                 "%s", oss.str().c_str());
        }
    }

    // 保存用来训练分类
    //  for(int i=0;i<armor_imgs.size();i++){
    //    cv::imwrite("./Label/fenqu_armor/" + std::to_string(armor_result[i]) + "/" + std::to_string(count_img++) + ".jpg", armor_imgs[i]);
    //  }

    int armor_idx = 0;
    for (auto& car : cars) {
        if (car.armors.size() == 0) {
            continue;
        }
        for (auto& armor : car.armors) {
            armor.class_label =
                (armor_idx < static_cast<int>(armor_result.size()))
                    ? armor_result[armor_idx++] : 0;
        }
    }
    int ci_main = 0;
    for (auto& car : cars) {
        const int ci = ci_main++;
        cv::Rect max_rect;
        float    max_confidence = 0;
        bool     armor_identified = false;
        if (car.armors.size() > 0) {
            for (auto& armor : car.armors) {
                if (armor_class_to_slot(armor.class_label) >= 0 &&
                    armor.confidence > max_confidence) {
                    max_rect = cv::Rect(
                        armor.left + car.car.left, armor.top + car.car.top,
                        armor.right - armor.left, armor.bottom - armor.top);
                    max_confidence = armor.confidence;
                    car.number = armor_class_to_slot(armor.class_label);
                    car.number_reliable = (car.number >= 0);
                    armor_identified = true;
                    car.armor_rect = max_rect;
                }
            }
        }
        car.armor_identified = armor_identified;
        car.center = cv::Point2f(
            car.car_rect.x + car.car_rect.width * 0.5f,
            car.car_rect.y + car.car_rect.height * 1.0f);

        auto color_from_rect = [&](const cv::Rect& r) {
            auto safe = getSafeRect(img, const_cast<cv::Rect&>(r));
            auto m = img(safe);
            return getColor(m);
        };
        if (armor_identified) {
            // Prefer the tight armor crop; fall back to the whole car when unknown.
            car.color = color_from_rect(max_rect);
            if (car.color == 1) car.color = color_from_rect(car.car_rect);
        } else {
            // No armor: still vote for color from the car ROI; tracking history
            // can recover a stable identity later.
            car.color  = color_from_rect(car.car_rect);
            car.number = -1;
        }
        // NOTE: per-car detect_result fill removed (was overwritten by the
        // track-smoothed `smoothed` message at the end of this callback).
    }
    // ---- Temporal IoU tracking: smooth color/number over frames ----
    const int n_cars = static_cast<int>(cars.size());
    std::vector<int> car_to_track(n_cars, -1);
    const int n_tracks_before = static_cast<int>(tracks_.size());
    std::vector<bool> track_matched(n_tracks_before, false);

    for (int ci = 0; ci < n_cars; ++ci) {
        // Allow ALL detected cars to match existing tracks (even unidentified ones).
        // This prevents nearby robots from losing their track when armor detection
        // temporarily fails. New track creation is gated later.
        float best_iou = 0.3F;
        int   best_ti  = -1;
        for (int ti = 0; ti < static_cast<int>(tracks_.size()); ++ti) {
            if (track_matched[ti]) continue;
            // 颜色互斥：已确认颜色的 track 不与异色 detection 匹配
            if (cars[ci].color != 1 && tracks_[ti].hit_count >= 3) {
                int tc = tracks_[ti].best_color();
                if (tc != 1 && tc != cars[ci].color) continue;
            }
            const auto& tb = tracks_[ti].last_box;
            const auto& cb = cars[ci].car_rect;
            float ix = static_cast<float>(std::max(0, std::min(tb.x+tb.width,  cb.x+cb.width)  - std::max(tb.x, cb.x)));
            float iy = static_cast<float>(std::max(0, std::min(tb.y+tb.height, cb.y+cb.height) - std::max(tb.y, cb.y)));
            float inter = ix * iy;
            float uni   = static_cast<float>(tb.area() + cb.area()) - inter;
            float iou   = (uni > 0) ? inter / uni : 0.0F;
            if (iou > best_iou) { best_iou = iou; best_ti = ti; }
        }
        if (best_ti >= 0) {
            car_to_track[ci] = best_ti;
            track_matched[best_ti] = true;
        }
    }

    // Update matched tracks, create new ones
    for (int ci = 0; ci < n_cars; ++ci) {
        int clr = cars[ci].color;
        int num = cars[ci].number;
        int ti  = car_to_track[ci];
        if (ti < 0) {
            // Only create a new track if at least color or number is known
            if (clr == 1 && num < 0) continue;
            CarTrack t;
            t.last_box = cars[ci].car_rect;
            t.smooth_box = cars[ci].car_rect;
            t.smooth_armor_box = cars[ci].armor_rect;
            t.has_armor_box = cars[ci].armor_identified;
            t.hit_count = 1;
            t.bot_id = next_bot_id_++;  // assign stable ID once at birth
            if (clr == 0 || clr == 2) t.color_hist[clr]++;
            if (num >= 0 && num < kRobotSlotCount) t.num_hist[num]++;
            tracks_.push_back(t);
            ti = static_cast<int>(tracks_.size()) - 1;
            car_to_track[ci] = ti;
        } else {
            tracks_[ti].last_box   = cars[ci].car_rect;
            tracks_[ti].smooth_box = cars[ci].car_rect;
            if (cars[ci].armor_identified) {
                tracks_[ti].smooth_armor_box = cars[ci].armor_rect;
                tracks_[ti].has_armor_box = true;
            }
            tracks_[ti].miss_count = 0;
            tracks_[ti].hit_count++;
            if (clr == 0 || clr == 2) tracks_[ti].color_hist[clr]++;
            if (num >= 0 && num < kRobotSlotCount) tracks_[ti].num_hist[num]++;
            // Cap history depth to avoid stale bias
            int total_c = tracks_[ti].color_hist[0] + tracks_[ti].color_hist[1] + tracks_[ti].color_hist[2];
            if (total_c > track_hist_depth_) {
                for (auto& v : tracks_[ti].color_hist) v = std::max(0, v - 1);
            }
            int total_n = 0; for (auto v : tracks_[ti].num_hist) total_n += v;
            if (total_n > track_hist_depth_) {
                for (auto& v : tracks_[ti].num_hist) v = std::max(0, v - 1);
            }
        }
        cars[ci].track_index = ti;
        cars[ci].car_rect = tracks_[ti].smooth_box;
        if (cars[ci].armor_identified && tracks_[ti].has_armor_box) {
            cars[ci].armor_rect = tracks_[ti].smooth_armor_box;
        }
        cars[ci].center = cv::Point2f(
            cars[ci].car_rect.x + cars[ci].car_rect.width * 0.5f,
            cars[ci].car_rect.y + cars[ci].car_rect.height * 1.0f);
        // Override single-frame detection with track consensus when confirmed
        if (ti >= 0 && tracks_[ti].hit_count >= 2) {
            int bc = tracks_[ti].best_color();
            int bn = tracks_[ti].best_number();
            if (bc != 1) cars[ci].color = bc;
            if (bn >= 0 && bn < kRobotSlotCount) cars[ci].number = bn;
        }
    }
    // Increment miss_count for unmatched tracks (do NOT erase yet — indices
    // must stay stable for the hold loop below).  track_matched[ti] is the
    // single source of truth (kept in sync with car_to_track during matching).
    for (int ti = n_tracks_before - 1; ti >= 0; --ti) {
        if (!track_matched[ti]) tracks_[ti].miss_count++;
    }

    // Hold recently missed tracks for drawing/output continuity.
    for (int ti = 0; ti < static_cast<int>(tracks_.size()); ++ti) {
        const bool matched_this_frame =
            (ti < static_cast<int>(track_matched.size())) && track_matched[ti];
        if (matched_this_frame) continue;
        if (tracks_[ti].miss_count <= 0 || tracks_[ti].miss_count > track_hold_miss_) continue;
        Car held;
        held.car_rect = tracks_[ti].smooth_box;
        held.armor_rect = tracks_[ti].smooth_armor_box;
        held.armor_identified = tracks_[ti].has_armor_box;
        held.track_index = ti;
        held.color = tracks_[ti].best_color();
        held.number = tracks_[ti].best_number();
        held.is_held = true;
        if (held.number >= 0 && held.number < kRobotSlotCount) {
            held.number_reliable = true;
            held.center = cv::Point2f(
                held.car_rect.x + held.car_rect.width * 0.5f,
                held.car_rect.y + held.car_rect.height * 1.0f);
            cars.push_back(held);
        }
    }

    vision_interface::msg::DetectResult smoothed;
    smoothed.header.stamp = detect_result.header.stamp;
    for (int ci = 0; ci < static_cast<int>(cars.size()); ++ci) {
        const int c = cars[ci].color;
        const int n = cars[ci].number;
        if (n < 0 || n >= kRobotSlotCount) continue;
        const int ti = cars[ci].track_index;
        const int bid = (ti >= 0 && ti < static_cast<int>(tracks_.size()))
                        ? tracks_[ti].bot_id : -1;
        if (c == 0) {
            smoothed.blue_x[n] = cars[ci].center.x; smoothed.blue_y[n] = cars[ci].center.y;
            smoothed.blue_w[n] = static_cast<float>(cars[ci].car_rect.width);
            smoothed.blue_h[n] = static_cast<float>(cars[ci].car_rect.height);
            smoothed.blue_bot_id[n] = bid;
        }
        if (c == 2) {
            smoothed.red_x[n]  = cars[ci].center.x; smoothed.red_y[n]  = cars[ci].center.y;
            smoothed.red_w[n]  = static_cast<float>(cars[ci].car_rect.width);
            smoothed.red_h[n]  = static_cast<float>(cars[ci].car_rect.height);
            smoothed.red_bot_id[n] = bid;
        }
    }
    detect_result = smoothed;

    // Erase expired tracks AFTER building the smoothed message, so that
    // cars[ci].track_index remains valid during bot_id lookup.
    for (int ti = static_cast<int>(tracks_.size()) - 1; ti >= 0; --ti) {
        if (tracks_[ti].miss_count > track_max_miss_)
            tracks_.erase(tracks_.begin() + ti);
    }

    detect_result.header.stamp = msg->header.stamp;
    pub->publish(detect_result);
    std::chrono::steady_clock::time_point end =
        std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used =
        std::chrono::duration_cast<std::chrono::duration<double>>(end -
                                                                  begin);
    ++processed_frame_count_;
    last_process_sec_ = this->get_clock()->now().seconds();
    const double detect_ms = time_used.count() * 1000.0;
    if (detect_time_ema_ms_ <= 0.0) {
        detect_time_ema_ms_ = detect_ms;
    } else {
        detect_time_ema_ms_ = detect_time_ema_ms_ * 0.9 + detect_ms * 0.1;
    }
    max_detect_time_ms_ = std::max(max_detect_time_ms_, detect_ms);
    if (last_process_sec_ - last_detect_diag_sec_ >= 2.0) {
        last_detect_diag_sec_ = last_process_sec_;
        const double run_s = std::max(0.001, last_process_sec_ - first_image_sec_);
        const double recv_fps = static_cast<double>(received_frame_count_) / run_s;
        const double proc_fps = static_cast<double>(processed_frame_count_) / run_s;
        const char *level = "正常";
        if (detect_time_ema_ms_ > 100.0 || stale_drop_count_ > 0) {
            level = "严重偏慢";
        } else if (detect_time_ema_ms_ > 60.0 || proc_fps < 10.0) {
            level = "偏慢";
        }
        RCLCPP_INFO(this->get_logger(),
            "【赛场体检-识别】%s：收到FPS=%.1f 处理FPS=%.1f 最近耗时=%.1fms 平均耗时=%.1fms 最大耗时=%.1fms 收到=%llu 处理=%llu 过期丢帧=%llu 限流丢帧=%llu。若平均>60ms或过期丢帧增加，优先降分辨率/关SAHI/关录包/查GPU占用。",
            level, recv_fps, proc_fps, detect_ms, detect_time_ema_ms_, max_detect_time_ms_,
            static_cast<unsigned long long>(received_frame_count_),
            static_cast<unsigned long long>(processed_frame_count_),
            static_cast<unsigned long long>(stale_drop_count_),
            static_cast<unsigned long long>(throttle_drop_count_));
    }
    static uint64_t detect_perf_counter = 0;
    ++detect_perf_counter;
    if ((detect_perf_counter % 60) == 0) {
        RCLCPP_INFO(this->get_logger(), "Detect Time: %.2f ms", detect_ms);
    }
    show_detect_view("DETECT");
}
}  // namespace tdt_radar
RCLCPP_COMPONENTS_REGISTER_NODE(tdt_radar::Detect)
