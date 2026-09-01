#include "cluster.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace tdt_radar {

// ── 物理常量（不暴露为 ROS 参数）─────────────────────────────────────────
// 坐标系: rm_frame, X∈[0,28], Y∈[0,15], Z 朝上。
// 输入点云已经过 dynamic_cloud 的 ROI/动态差分, 这里只做空间聚类。
namespace {

// 场地包围 (rm_frame, 略外扩 0.5m 余量)
constexpr float kFieldXMin = -0.5F;
constexpr float kFieldXMax = 28.5F;
constexpr float kFieldYMin = -0.5F;
constexpr float kFieldYMax = 15.5F;

// 机器人 Z 通带：去地面(z>3cm)，砍超高(hero 1.2m + 高地 0.6m + 0.2m 余量)
constexpr float kZMin = 0.03F;
constexpr float kZMax = 2.00F;

// 占用栅格分辨率: 10cm
//   RM2026 底盘最大 1.2m, 机器人间最小间距 ≈ 0.3m -> 10cm 栅格保证可分。
//   Mid-70 在 25m 处相邻点距 ≈ 0.12m, 10cm 栅格不会把单个机器人切碎(因为有 z-OR)。
constexpr float kGridRes  = 0.10F;
constexpr int   kGridCols = static_cast<int>((kFieldXMax - kFieldXMin) / kGridRes + 0.5F);  // 290
constexpr int   kGridRows = static_cast<int>((kFieldYMax - kFieldYMin) / kGridRes + 0.5F);  // 160

// 单 cell 视为「有效占用」所需最小 hit 数。
//   近场每机器人投影到 ~50 cell, 远场 (>20m) 可能只剩 2~3 点 -> 阈值 1。
//   差分阶段已经把地面/墙剔了, 误报概率极低, 用 1 即可。
constexpr int   kCellHitMin = 1;

// 候选机器人足迹 sanity：投影连通域 cell 数对应的 XY 面积。
//   RM2026 hero 1.2×1.2 = 1.44 m² → 上限 1.6×1.6=2.56m² → 256 cell
//   下限 0.5×0.5 = 0.25m² 上方 (单 Mid-70 点足以触发 1 cell, 故下限取 1)
//   下限太严会丢远场, 太松会留毛刺; 用 1 保留全部候选, 交给 KF 阶段汰除。
constexpr int   kClusterCellMin = 1;
constexpr int   kClusterCellMax = 256;

// 物理尺寸 sanity（rm_frame 直接量）
constexpr float kClusterMinZExt  = 0.03F;
constexpr float kClusterMaxZExt  = 1.30F;
constexpr float kClusterMaxXYExt = 1.40F;

// trimmed-mean 比例：剔除最远 15% 点再求平均, 抑制远场单点跳变。
constexpr float kCentroidTrimFrac = 0.15F;

inline int cellX(float x) { return static_cast<int>((x - kFieldXMin) / kGridRes); }
inline int cellY(float y) { return static_cast<int>((y - kFieldYMin) / kGridRes); }

}  // namespace

Cluster::Cluster(const rclcpp::NodeOptions& node_options)
    : rclcpp::Node("cluster", node_options)
{
    log_cluster_timing_ = declare_parameter<bool>("log_cluster_timing", log_cluster_timing_);

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox/lidar_dynamic", 10,
        std::bind(&Cluster::callback, this, std::placeholders::_1));
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar_cluster", 10);

    RCLCPP_INFO(get_logger(),
        "Cluster ready: grid=%dx%d (res=%.2fm), cell_hit≥%d, footprint∈[%d,%d] cells",
        kGridCols, kGridRows, kGridRes, kCellHitMin, kClusterCellMin, kClusterCellMax);
}

void Cluster::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    const auto t1 = std::chrono::steady_clock::now();

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    if (cloud.empty()) {
        // 仍然发空消息, 维持下游话题心跳
        sensor_msgs::msg::PointCloud2 empty;
        pcl::toROSMsg(cloud, empty);
        empty.header = msg->header;
        empty.header.frame_id = "rm_frame";
        pub_->publish(empty);
        return;
    }

    // ── 1. 投到 2D 占用栅格（10cm），同步记录每 cell 的 hit 索引列表 ──
    //    用 flat row-major; cell 索引 = ry * kGridCols + rx
    const int total_cells = kGridCols * kGridRows;
    std::vector<uint16_t> hit_count(total_cells, 0);
    std::vector<std::vector<int>> cell_points(total_cells);  // 索引列表

    for (size_t i = 0; i < cloud.points.size(); ++i) {
        const auto& p = cloud.points[i];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        if (p.z < kZMin || p.z > kZMax) continue;
        const int cx = cellX(p.x), cy = cellY(p.y);
        if (cx < 0 || cx >= kGridCols || cy < 0 || cy >= kGridRows) continue;
        const int idx = cy * kGridCols + cx;
        if (hit_count[idx] < UINT16_MAX) ++hit_count[idx];
        cell_points[idx].push_back(static_cast<int>(i));
    }

    // ── 2. 8-邻域 BFS 连通域 (栅格化机器人)──────────────────────────────
    // 8-邻域 (含对角) 保证 RM2026 1.2m × 1.2m 底盘 (~12 cell) 在斜向 lidar
    // sampling 下不会被切成两块; 4-邻域在对角间隙 1cell 时会误分。
    std::vector<int> label(total_cells, -1);
    std::vector<std::vector<int>> cluster_cells;  // 每个 cluster 的 cell idx 列表
    std::vector<int> stack;
    stack.reserve(64);
    for (int ry = 0; ry < kGridRows; ++ry) {
        for (int rx = 0; rx < kGridCols; ++rx) {
            const int idx = ry * kGridCols + rx;
            if (hit_count[idx] < kCellHitMin || label[idx] != -1) continue;
            const int cl_id = static_cast<int>(cluster_cells.size());
            cluster_cells.emplace_back();
            auto& cells = cluster_cells.back();
            stack.clear();
            stack.push_back(idx);
            label[idx] = cl_id;
            while (!stack.empty()) {
                const int cur = stack.back();
                stack.pop_back();
                cells.push_back(cur);
                const int cy = cur / kGridCols, cx = cur - cy * kGridCols;
                static constexpr std::array<int, 8> dx{ 1, -1, 0, 0,  1,  1, -1, -1};
                static constexpr std::array<int, 8> dy{ 0,  0, 1,-1,  1, -1,  1, -1};
                for (int k = 0; k < 8; ++k) {
                    const int nx = cx + dx[k], ny = cy + dy[k];
                    if (nx < 0 || nx >= kGridCols || ny < 0 || ny >= kGridRows) continue;
                    const int ni = ny * kGridCols + nx;
                    if (label[ni] != -1) continue;
                    if (hit_count[ni] < kCellHitMin) continue;
                    label[ni] = cl_id;
                    stack.push_back(ni);
                }
            }
        }
    }

    // ── 3. 物理 sanity + 4. trimmed-mean 质心 ──────────────────────────
    pcl::PointCloud<pcl::PointXYZ> out;
    out.points.reserve(cluster_cells.size());
    size_t dropped = 0;

    for (const auto& cells : cluster_cells) {
        // cell 数 (足迹) 早筛
        if (static_cast<int>(cells.size()) < kClusterCellMin ||
            static_cast<int>(cells.size()) > kClusterCellMax) {
            ++dropped;
            continue;
        }

        // 收集所有 hit 点
        std::vector<int> pt_idx;
        pt_idx.reserve(cells.size() * 4);
        for (int ci : cells) {
            const auto& v = cell_points[ci];
            pt_idx.insert(pt_idx.end(), v.begin(), v.end());
        }
        if (pt_idx.empty()) { ++dropped; continue; }

        // 物理 XY/Z extent (rm_frame 直接量)
        float x_min =  1e9F, x_max = -1e9F;
        float y_min =  1e9F, y_max = -1e9F;
        float z_min =  1e9F, z_max = -1e9F;
        for (int j : pt_idx) {
            const auto& p = cloud.points[j];
            x_min = std::min(x_min, p.x); x_max = std::max(x_max, p.x);
            y_min = std::min(y_min, p.y); y_max = std::max(y_max, p.y);
            z_min = std::min(z_min, p.z); z_max = std::max(z_max, p.z);
        }
        const float xy_ext = std::max(x_max - x_min, y_max - y_min);
        const float z_ext  = z_max - z_min;
        if (z_ext  < kClusterMinZExt ||
            z_ext  > kClusterMaxZExt ||
            xy_ext > kClusterMaxXYExt) {
            ++dropped;
            continue;
        }

        // 初步 centroid (mean), 再剔除最远 15% 后重算 mean
        double cx = 0.0, cy = 0.0, cz = 0.0;
        for (int j : pt_idx) {
            cx += cloud.points[j].x;
            cy += cloud.points[j].y;
            cz += cloud.points[j].z;
        }
        const double inv_n = 1.0 / static_cast<double>(pt_idx.size());
        cx *= inv_n; cy *= inv_n; cz *= inv_n;

        if (pt_idx.size() >= 4) {
            // 按到初步质心的平方距离排序，剔除最远 trim_frac
            std::vector<std::pair<float, int>> rank;
            rank.reserve(pt_idx.size());
            for (int j : pt_idx) {
                const float dx = cloud.points[j].x - static_cast<float>(cx);
                const float dy = cloud.points[j].y - static_cast<float>(cy);
                const float dz = cloud.points[j].z - static_cast<float>(cz);
                rank.emplace_back(dx*dx + dy*dy + dz*dz, j);
            }
            std::sort(rank.begin(), rank.end(),
                      [](const auto& a, const auto& b){ return a.first < b.first; });
            const size_t keep = std::max<size_t>(2,
                static_cast<size_t>(std::round(rank.size() * (1.0F - kCentroidTrimFrac))));
            cx = cy = cz = 0.0;
            for (size_t k = 0; k < keep; ++k) {
                const auto& p = cloud.points[rank[k].second];
                cx += p.x; cy += p.y; cz += p.z;
            }
            const double inv_k = 1.0 / static_cast<double>(keep);
            cx *= inv_k; cy *= inv_k; cz *= inv_k;
        }

        pcl::PointXYZ c;
        c.x = static_cast<float>(cx);
        c.y = static_cast<float>(cy);
        c.z = static_cast<float>(cz);
        out.points.push_back(c);
    }
    out.width  = out.points.size();
    out.height = 1;
    out.is_dense = true;

    // ── 5. Publish ─────────────────────────────────────────────────────
    sensor_msgs::msg::PointCloud2 ros_out;
    pcl::toROSMsg(out, ros_out);
    ros_out.header = msg->header;
    ros_out.header.frame_id = "rm_frame";
    pub_->publish(ros_out);

    if (log_cluster_timing_) {
        const auto t2 = std::chrono::steady_clock::now();
        RCLCPP_INFO(get_logger(),
            "Cluster: %zu pts → %zu clusters (dropped %zu), %.2f ms",
            cloud.size(), out.size(), dropped,
            std::chrono::duration<double, std::milli>(t2 - t1).count());
    }
}

}  // namespace tdt_radar

RCLCPP_COMPONENTS_REGISTER_NODE(tdt_radar::Cluster)
