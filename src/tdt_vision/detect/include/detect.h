#ifndef RADAR_DETECT_H
#define RADAR_DETECT_H

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "classify.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "NvidiaInterface.hpp"
#include "opencv2/opencv.hpp"
#include "vision_interface/msg/detect_result.hpp"
#include "yolos.hpp"
#include "BaseInfer.hpp"
#include "sr_infer.h"
#include <fstream>
#include <array>
#include "radar_utils.h"
namespace tdt_radar {

struct CarTrack {
    cv::Rect last_box;
    cv::Rect smooth_box;
    cv::Rect smooth_armor_box;
    bool has_armor_box = false;
    std::array<int, 3> color_hist = {0, 0, 0};   // votes: 0=blue 1=unknown 2=red
    std::array<int, 5> num_hist   = {0,0,0,0,0};
    int miss_count = 0;
    int hit_count  = 0;
    int bot_id     = -1;  // stable per-track ID for short-term re-association
    int best_color()  const {
        return static_cast<int>(
            std::max_element(color_hist.begin(), color_hist.end()) - color_hist.begin());
    }
    int best_number() const {
        auto it = std::max_element(num_hist.begin(), num_hist.end());
        if (*it == 0) return -1;
        return static_cast<int>(it - num_hist.begin());
    }
};

class Detect final : public rclcpp::Node {
public:
    explicit Detect(const rclcpp::NodeOptions& options);
    void callback(const std::shared_ptr<sensor_msgs::msg::Image> msg);
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;

private:
    std::shared_ptr<Infer<yolo::BoxArray>>     yolo;
    std::shared_ptr<Infer<yolo::BoxArray>>     armor_yolo;
    std::shared_ptr<Infer<int>> classifier;
    rclcpp::Publisher<vision_interface::msg::DetectResult>::SharedPtr pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;

    int         debug = 0;
    bool        detect_view_ = false;
    double      detect_view_scale_ = 1.0;
    bool        force_draw_result_ = true;
    double      max_process_fps_ = 20.0;
    int         drop_stale_ms_ = 250;
    int64_t     last_process_ts_ns_ = 0;
    uint64_t    dropped_frame_count_ = 0;
    uint64_t    received_frame_count_ = 0;
    uint64_t    processed_frame_count_ = 0;
    uint64_t    stale_drop_count_ = 0;
    uint64_t    throttle_drop_count_ = 0;
    double      last_detect_diag_sec_ = -1e9;
    double      first_image_sec_ = -1e9;
    double      last_image_sec_ = -1e9;
    double      last_process_sec_ = -1e9;
    double      detect_time_ema_ms_ = 0.0;
    double      max_detect_time_ms_ = 0.0;
    std::string yolo_path;
    std::string yolo_onnx;
    std::string armor_path;
    std::string classify_path;

    bool        sahi_enable_ = false;
    float       sahi_scale_ = 1.0f;
    float       sahi_overlap_ = 0.2f;
    float       sahi_merge_iou_ = 0.5f;
    // 中间条带 SAHI: 只在图像中间垂直区域 [y_band_top, y_band_bot] 切横向 tile.
    // 上端=天花板, 下端=近场杂物, 都不切. 全图 YOLO base pass 仍兜底所有距离.
    float       sahi_y_band_top_ = 0.0f;   // 0.0=不裁上端; 0.25=跳过上 25%
    float       sahi_y_band_bot_ = 1.0f;   // 1.0=不裁下端; 0.75=跳过下 25%

    bool        sr_enable_ = false;
    int         sr_min_dim_ = 150;
    std::string sr_engine_;
    int         sr_max_dim_ = 128;
    std::shared_ptr<SRInfer> sr_infer_;

    std::vector<CarTrack> tracks_;
    int next_bot_id_      = 0;    // monotonically increasing; wraps at 2^31
    int track_max_miss_   = 5;
    int track_hist_depth_ = 8;
    int track_hold_miss_ = 1;

    // 3D cuboid overlay: load camera intrinsics + solvePnP extrinsics, project
    // bbox bottom centre to ground, build an axis-aligned cuboid in referee
    // frame and re-project its 8 corners.  Off by default (set draw_3d_box=true).
    bool        draw_3d_box_ = false;
    bool        box3d_valid_ = false;
    cv::Mat     box3d_K_, box3d_D_, box3d_rvec_, box3d_tvec_;
    cv::Mat     box3d_R_, box3d_Rt_, box3d_C_;
    float       box3d_map_height_ = 15.0F;
    bool        box3d_referee_ = true;  // calibration points in referee frame
};
class Car {
public:
    cv::Rect       car_rect;
    cv::Rect       armor_rect;
    yolo::Box      car{};
    yolo::BoxArray armors;
    cv::Point2f    center;
    cv::Rect       center_rect;
    int            number = -1;
    int            color = 1;
    bool           number_reliable = false;
    bool           armor_identified = false;
    int            track_index = -1;
    float          detection_confidence = 0.0f;
    bool           has_detection_confidence = false;
    bool           is_held = false;  // true if synthesized from a missed track (not a real detection this frame)
};
}  // namespace tdt_radar

#endif
