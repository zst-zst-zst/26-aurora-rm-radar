#ifndef RADAR_RESOLVE_H
#define RADAR_RESOLVE_H

#include <memory>
#include <mutex>
#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "opencv2/opencv.hpp"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "radar_utils.h"
#include "vision_interface/msg/detect_result.hpp"
#include "vision_interface/msg/radar2_sentry.hpp"
namespace tdt_radar {

class Resolve final : public rclcpp::Node {
public:
    explicit Resolve(const rclcpp::NodeOptions& options);
    void callback(const std::shared_ptr<geometry_msgs::msg::Vector3> msg);
    void DetectCallback(
        const vision_interface::msg::DetectResult::SharedPtr msg);
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr point_sub;
    rclcpp::Subscription<vision_interface::msg::DetectResult>::SharedPtr
        detect_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
            cluster_sub_;
    void ClusterCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    std::unique_ptr<parser> parser_;
    std::mutex            cluster_mu_;
    std::vector<cv::Point2f> latest_clusters_;  // world (x,y) of latest LiDAR clusters
    float frustum_pixel_gate_ = 150.0F; // pixels in original image space (brand-dependent)
    // Consistency gate (metres): if |lidar_pos - camera_pos| > gate, treat cluster as mis-match
    // and fall back to camera.  0 = disabled (legacy: always trust LiDAR).
    float lidar_consistency_gate_ = 0.f;
    // Camera-primary fusion: accumulate EMA(lidar - camera) as extrinsics bias correction
    bool        lidar_calibrate_camera_ = false;
    cv::Point2f cam_bias_{0.f, 0.f};  // EMA bias applied to all camera positions
    float       cam_bias_alpha_  = 0.05F; // EMA learning rate (lower = slower but more stable)
    int         cam_bias_samples_ = 0;    // warm-up counter for cold-start

private:
    float convert_parsed_y(float parsed_y, bool use_legacy_plus) const;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub;
    rclcpp::Publisher<vision_interface::msg::DetectResult>::SharedPtr
        pub_radar;
    float       arena_width_ = 28.0F;
    float       arena_height_ = 15.0F;
    bool        detect_use_legacy_plus_y_ = true;
    float       pixel_scale_x_ = 1.0F;  // yolo.cu already maps boxes back to original image coords
    float       pixel_scale_y_ = 1.0F;
};

class map_car {
public:
    float x;
    float y;
    int   id;
};
}  // namespace tdt_radar

#endif
