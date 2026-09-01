#ifndef DYNAMIC_CLOUD_H
#define DYNAMIC_CLOUD_H

#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <vision_interface/msg/match_info.hpp>
#include <vision_interface/msg/radar_warn.hpp>

namespace tdt_radar {

// ── Map-diff via 2D height grid (替代 KdTree) ──────────────────────────
// 每个 5cm × 5cm 单元记录静态地图在该 XY 上的最大 Z (即「地形顶」)。
// 运行时对一帧落入栅格内的点: 若 p.z > max_z(cell) + eps  → 动态点。
// 优势: O(1) 查询 vs KdTree O(log N), 帧耗下降 5-10×;
//      容差自动随地形变化(高地/障碍物/墙)而变, 不再需要全局阈值。
// 单元尺寸 5cm 远小于机器人尺寸, 保证机器人主体不会落入「桌面」cell。
//
// ── 累积策略 cell-TTL (替代 N 帧滑窗) ──────────────────────────────────
// 不再固定累积 N 帧, 改为每个 cell 记录最近一次 hit 时间戳;
// 输出帧只发 (now - last_hit < 300 ms) 的 cell 内最新点。
// 优势: 与机器人速度解耦, 不会拖尾 (3 m/s 机器人 N=3 滑窗会有 ~0.9 m 残影)。
//
// 所有几何/算法参数均为物理常量, 不暴露为 ROS 参数。
class DynamicCloud : public rclcpp::Node {
public:
    explicit DynamicCloud(const rclcpp::NodeOptions& options);
    ~DynamicCloud() override = default;

private:
    void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void on_match(const vision_interface::msg::MatchInfo::SharedPtr msg);
    bool lookup_tf(const rclcpp::Time& stamp, Eigen::Affine3f& out_T);
    // 从 config/map/field_mesh.bin (1cm CAD 高度网格) 加载 max-Z 栅格,
    // 头部 6×float64 = (x_min, y_min, nx, ny, res, reserved) + ny×nx float32 row-major。
    bool load_height_grid_bin(const std::string& path);

    // ── 订阅/发布 ───────────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr  sub_cloud_;
    rclcpp::Subscription<vision_interface::msg::MatchInfo>::SharedPtr sub_match_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr     pub_dyn_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr     pub_other_;
    rclcpp::Publisher<vision_interface::msg::RadarWarn>::SharedPtr  pub_warn_;

    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ── Height-grid 静态地图 (1cm CAD 网格) ────────────────────────────
    // row-major: idx = cy * cols + cx
    std::vector<float> map_max_z_;     // 每 cell 的 CAD max-Z (NaN 表示无数据)
    float grid_x_min_ = 0.0F;
    float grid_y_min_ = 0.0F;
    float grid_res_   = 0.01F;         // 1cm
    int   grid_cols_  = 0;
    int   grid_rows_  = 0;

    // ── cell-TTL 累积 ──────────────────────────────────────────────────
    // 对每个动态 cell 存「最近一次命中时刻 + 最近一帧点」
    struct CellHit {
        double t_sec;
        pcl::PointXYZ p;
    };
    std::vector<CellHit> cell_hit_;    // size = grid_cols_ * grid_rows_

    // ── 仅保留为 ROS 参数的运行时项 ─────────────────────────────────────
    int   self_color_   = -1;  // -1 auto / 0 blue / 2 red (当前不主动场地翻转)
};

}  // namespace tdt_radar

#endif
