#ifndef RADAR_CALIBRATE_H
#define RADAR_CALIBRATE_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/photo.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include "radar_utils.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace tdt_radar {
static cv::Mat                  cvimage_;
static cv::Mat                  orig_image_;
static std::vector<cv::Point2f> pick_points;
static cv::Mat                  camera_matrix;
static cv::Mat                  dist_coeffs;
static cv::Mat                  rvec;
static cv::Mat                  tvec;
static bool                     is_calibrating = false;
static int                      orig_width_    = 3072; // default placeholder
static int                      orig_height_   = 2048; // default placeholder
static int                      display_width_ = 1536; // default placeholder
static int                      display_height_= 1024; // default placeholder
// ROI half-size 与 wasd 微调步长按图像宽度自适应：
//   6MP (3072): roi_half ≈ 102, step = 2
//   low-res (1280): roi_half ≈ 50, step = 1
static int                      roi_half_      = 50;   // updated per frame
static int                      pick_step_     = 1;    // updated per frame

class Calibrate final : public rclcpp::Node {
public:
    std::vector<cv::Point3f> real_points;
    std::string              out_matrix_path_;
    float                    map_height_ = 15.0F;
    bool                     points_in_referee_frame_ = true;

    std::string team_label_;     // "red" / "blue" / ""，用于标定结果打印

    explicit Calibrate(const rclcpp::NodeOptions& options);
    // 标定只订压缩流：实时和回放都有 compressed_image，避免双订阅抢键
    void compressed_callback(
        const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr
                                                   compressed_image_sub;
    void                                           solve();
    parser*                                        parser_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    geometry_msgs::msg::TransformStamped           transformStamped;
};
void mousecallback(int event, int x, int y, int flags, void* userdata);

}  // namespace tdt_radar
#endif
