#include "dynamic_cloud.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include <cstdio>
#include <cstring>
#include <fstream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace tdt_radar {

// ── 物理常量 (CAD + RM2026 规则书 V1.4.2 / 场地 V1.2.0) ─────────────────
namespace {

// 动态点判定: p.z > map_max_z(cell) + diff_eps(z)
//   diff_eps(z) = max(kDiffEpsAbs, kDiffEpsRel * z)
//
// 参数标定 (2026-05): 实测 ICP RMSE 0.27-0.48m, Mid-70 远端 z 噪声 5-10cm,
// CAD vs 真实场地 ±5cm. 总残差约 20-30cm. 之前用 5cm 容差导致 98% 真实
// 静态点被误判为动态, cluster 把它们聚成假机器人污染下游 KF/minimap.
//   - kDiffEpsAbs = 25cm: 必须大于 ICP RMSE 上限以容纳配准误差
//   - kDiffEpsRel = 8%:   高地形 (飞坡 1m × 8% = 8cm 额外余量)
// 机器人高度 ≥ 0.5m, 仍远高于 0.25m 阈值, 不会被误删。
constexpr float kDiffEpsAbs = 0.25F;
constexpr float kDiffEpsRel = 0.08F;

// 默认高度网格路径 (1cm CAD 预计算), 脱机场景下仓库相对路径。
constexpr const char* kFieldMeshBin = "config/map/field_mesh.bin";

// cell-TTL: 300ms = 比赛感知节拍 (3 Hz 决策)
//   太长拖尾, 太短稀疏帧时丢轨。
constexpr double kCellTtlSec = 0.30;

// 地面/天花板裁剪
constexpr float kZGround = 0.03F;   // 略高于 cad 地面 (剔残点)
constexpr float kZSky    = 12.0F;   // 飞镖最高弹道 ~ 8m, 留余量

// Dart 检测: 飞行高度 > 3.6m (高于 CAD 中央 R3 高架 z_max=3.50m)
//   CAD 实测中央 dart-launch 基座顶部最高 3.497m, 必须 >3.6 才能与静态结构区分。
constexpr float kDartZMin = 3.6F;

// Fly 检测: 飞坡上方"airborne"高度 (区分"站在飞坡"和"飞行中")
//   CAD 实测飞坡顶 z=0.635m; 机器人站在坡上中心 ≈ 1.0m;
//   真正起跳后离地 (底部 > 0.3m above ramp) → 中心 z ≥ 1.5m。
constexpr float kFlyZMin = 1.5F;
constexpr float kFlyZMax = 3.0F;
constexpr float kFlyRedXMin  = 12.5F, kFlyRedXMax  = 15.5F;
constexpr float kFlyRedYMin  =  0.0F, kFlyRedYMax  =  2.0F;
constexpr float kFlyBlueXMin = 12.5F, kFlyBlueXMax = 15.5F;
constexpr float kFlyBlueYMin = 13.0F, kFlyBlueYMax = 15.0F;

inline int cellIdx(int cx, int cy, int cols) { return cy * cols + cx; }

}  // namespace

bool DynamicCloud::load_height_grid_bin(const std::string& path)
{
    std::ifstream f(path, std::ios::binary);
    if (!f.is_open()) {
        RCLCPP_ERROR(get_logger(), "Cannot open height-grid bin: %s", path.c_str());
        return false;
    }
    double hdr[6] = {};
    f.read(reinterpret_cast<char*>(hdr), sizeof(hdr));
    if (!f) {
        RCLCPP_ERROR(get_logger(), "Failed reading header from %s", path.c_str());
        return false;
    }
    grid_x_min_ = static_cast<float>(hdr[0]);
    grid_y_min_ = static_cast<float>(hdr[1]);
    grid_cols_  = static_cast<int>(hdr[2]);   // nx
    grid_rows_  = static_cast<int>(hdr[3]);   // ny
    grid_res_   = static_cast<float>(hdr[4]);
    const size_t N = static_cast<size_t>(grid_cols_) * static_cast<size_t>(grid_rows_);
    map_max_z_.assign(N, std::numeric_limits<float>::quiet_NaN());
    f.read(reinterpret_cast<char*>(map_max_z_.data()),
           static_cast<std::streamsize>(N * sizeof(float)));
    if (!f) {
        RCLCPP_ERROR(get_logger(), "Truncated height-grid bin: %s", path.c_str());
        map_max_z_.clear();
        return false;
    }
    cell_hit_.assign(N, CellHit{0.0, pcl::PointXYZ()});
    return true;
}

DynamicCloud::DynamicCloud(const rclcpp::NodeOptions& options)
    : rclcpp::Node("dynamic_cloud", options)
{
    // ── 运行时参数 (只留 self_color, 其他全部写死) ──────
    // 高度栅格从官方 STEP CAD 预计算 (1cm 网格, 19MB) 加载。
    // arena 尺寸完全由 bin 头部 (grid_x_min_/grid_y_min_/grid_cols_/grid_rows_) 决定。
    self_color_   = declare_parameter<int>("self_color", self_color_);

    if (!load_height_grid_bin(kFieldMeshBin)) {
        RCLCPP_FATAL(get_logger(),
            "Cannot load CAD height grid (%s); run tools/npz_to_heightgrid.py first.",
            kFieldMeshBin);
        return;
    }
    RCLCPP_INFO(get_logger(),
        "CAD height grid: %dx%d @ %.0fcm, origin=(%.2f, %.2f), cell-TTL=%.0fms, eps=max(%.0fcm, %.0f%% z)",
        grid_cols_, grid_rows_, grid_res_ * 100.0F,
        grid_x_min_, grid_y_min_,
        kCellTtlSec * 1000.0,
        kDiffEpsAbs * 100.0F, kDiffEpsRel * 100.0F);

    // ── TF / 订阅 / 发布 ───────────────────────────────────────────────
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox/lidar", rclcpp::SensorDataQoS(),
        std::bind(&DynamicCloud::on_cloud, this, std::placeholders::_1));

    sub_match_ = create_subscription<vision_interface::msg::MatchInfo>(
        "/match_info", 10,
        std::bind(&DynamicCloud::on_match, this, std::placeholders::_1));

    pub_dyn_   = create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar_dynamic", 10);
    pub_other_ = create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar_other", 10);
    pub_warn_  = create_publisher<vision_interface::msg::RadarWarn>("/lidar_detect", 10);
}

void DynamicCloud::on_match(const vision_interface::msg::MatchInfo::SharedPtr msg)
{
    // 仅更新自身颜色 (用于 fly 区域归属归类), 不做场地翻转 — kalman_filter 已在全局坐标。
    if (msg->self_color == 0 || msg->self_color == 2) self_color_ = msg->self_color;
}

bool DynamicCloud::lookup_tf(const rclcpp::Time& stamp, Eigen::Affine3f& out_T)
{
    try {
        // 用最新的 TF (Time(0)) 避免 ICP 慢节拍下「找不到精确时间戳」的问题
        // localization 在稳态期间是固定 TF, 时间误差不影响精度。
        (void)stamp;
        auto ts = tf_buffer_->lookupTransform(
            "rm_frame", "livox_frame", tf2::TimePointZero,
            tf2::durationFromSec(0.05));
        Eigen::Isometry3d Td = tf2::transformToEigen(ts.transform);
        out_T = Td.cast<float>();
        return true;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "TF rm_frame ← livox_frame unavailable: %s", ex.what());
        return false;
    }
}

void DynamicCloud::on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if (map_max_z_.empty()) return;  // map not yet loaded

    const auto t0 = std::chrono::steady_clock::now();
    const double now_sec = std::chrono::duration<double>(
        t0.time_since_epoch()).count();

    Eigen::Affine3f T;
    if (!lookup_tf(msg->header.stamp, T)) return;

    pcl::PointCloud<pcl::PointXYZ> raw;
    pcl::fromROSMsg(*msg, raw);
    if (raw.empty()) return;

    // ── ROI + 高度栅格差分 + Dart/Fly 检测 ────────────────────────────
    pcl::PointCloud<pcl::PointXYZ> other_cloud;  // 用于 /livox/lidar_other (调试/dart)
    other_cloud.points.reserve(raw.size() / 8);

    uint8_t dart_state = 0;
    uint8_t fly_state  = 0;

    for (const auto& p_in : raw.points) {
        if (!std::isfinite(p_in.x) || !std::isfinite(p_in.y) || !std::isfinite(p_in.z)) continue;
        // 变换到 rm_frame
        Eigen::Vector3f q = T * Eigen::Vector3f(p_in.x, p_in.y, p_in.z);
        const float x = q.x(), y = q.y(), z = q.z();

        // 超场或贴地一律丢 (场地边界由 CAD 网格 bbox 隐式决定)
        if (z < kZGround || z > kZSky) continue;
        if (x < grid_x_min_ || y < grid_y_min_) continue;

        // Dart 高空检测
        if (z > kDartZMin && z < kZSky) {
            dart_state = 1;
            other_cloud.points.push_back(pcl::PointXYZ(x, y, z));
            continue;  // dart 不进 dyn (避免污染机器人聚类)
        }

        // Fly 飞坡检测
        if (z > kFlyZMin && z < kFlyZMax) {
            const bool in_red  = (x > kFlyRedXMin  && x < kFlyRedXMax  &&
                                  y > kFlyRedYMin  && y < kFlyRedYMax);
            const bool in_blue = (x > kFlyBlueXMin && x < kFlyBlueXMax &&
                                  y > kFlyBlueYMin && y < kFlyBlueYMax);
            if (in_red)  fly_state = 1;  // 红方飞坡
            if (in_blue) fly_state = (fly_state == 1) ? 3 : 2;  // 1=红, 2=蓝, 3=双
        }

        // 高度栅格 map-diff (CAD 1cm 网格)
        const int cx = static_cast<int>((x - grid_x_min_) / grid_res_);
        const int cy = static_cast<int>((y - grid_y_min_) / grid_res_);
        if (cx < 0 || cx >= grid_cols_ || cy < 0 || cy >= grid_rows_) continue;
        const int idx = cellIdx(cx, cy, grid_cols_);
        const float mz = map_max_z_[idx];
        if (std::isnan(mz)) {
            // CAD 未覆盖的 cell (场地边缘洞): 要求 z > 0.15m 才算动态。
            if (z < kZGround + 0.15F) continue;
        } else {
            // 官方场地与 CAD 容许 5% 几何公差, 阈值随地形高度自适应。
            const float eps = std::max(kDiffEpsAbs, kDiffEpsRel * std::max(0.0F, mz));
            if (z <= mz + eps) continue;  // 静态
        }

        // 写入 cell-TTL: 该 cell 记录最近一次命中
        cell_hit_[idx] = CellHit{ now_sec, pcl::PointXYZ(x, y, z) };
    }

    // ── 收集 cell-TTL 仍活跃的点 → /livox/lidar_dynamic ────────────────
    pcl::PointCloud<pcl::PointXYZ> dyn_cloud;
    dyn_cloud.points.reserve(256);
    for (const auto& c : cell_hit_) {
        if (c.t_sec > 0.0 && (now_sec - c.t_sec) < kCellTtlSec)
            dyn_cloud.points.push_back(c.p);
    }
    dyn_cloud.width  = dyn_cloud.points.size();
    dyn_cloud.height = 1;
    dyn_cloud.is_dense = true;

    other_cloud.width  = other_cloud.points.size();
    other_cloud.height = 1;
    other_cloud.is_dense = true;

    // ── 发布 ──────────────────────────────────────────────────────────
    {
        sensor_msgs::msg::PointCloud2 out;
        pcl::toROSMsg(dyn_cloud, out);
        out.header.stamp = msg->header.stamp;
        out.header.frame_id = "rm_frame";
        pub_dyn_->publish(out);
    }
    {
        sensor_msgs::msg::PointCloud2 out;
        pcl::toROSMsg(other_cloud, out);
        out.header.stamp = msg->header.stamp;
        out.header.frame_id = "rm_frame";
        pub_other_->publish(out);
    }
    {
        vision_interface::msg::RadarWarn w;
        w.dart_state = dart_state;
        w.fly_state  = fly_state;
        w.engine_state = 0;
        w.hero_state = 0;
        pub_warn_->publish(w);
    }
}

}  // namespace tdt_radar

RCLCPP_COMPONENTS_REGISTER_NODE(tdt_radar::DynamicCloud)
