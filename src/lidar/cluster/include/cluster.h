#ifndef CLUSTER_H
#define CLUSTER_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace tdt_radar {

// ── 占用栅格连通域聚类 (替代欧氏 + 远场两 pass) ────────────────────────────
// 全部几何常量来自 RM2026 规则手册 V1.4.2 + 场地 V1.2.0：
//   场地外尺寸 28×15m, 机器人最高 hero 1.2m + 高地 0.6m, 底盘最大 1.2×1.2m。
// 聚类参数全部由物理常量推导，运行时无任何可调参数；仅留 log_timing 调试开关。
class Cluster : public rclcpp::Node {
public:
    explicit Cluster(const rclcpp::NodeOptions& node_options);
    ~Cluster() override = default;

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_;

    bool log_cluster_timing_ = false;  // 唯一保留的调试开关
};

}  // namespace tdt_radar

#endif
