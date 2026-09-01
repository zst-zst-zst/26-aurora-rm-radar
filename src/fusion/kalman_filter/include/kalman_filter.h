#pragma once

#include <map>
#include <string>
#include <utility>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <vision_interface/msg/detect_result.hpp>
#include <vision_interface/msg/match_info.hpp>
#include <vision_interface/msg/radar2_sentry.hpp>
#include <vision_interface/msg/radar_warn.hpp>
#include "filter_plus.h"
#include "opencv2/opencv.hpp"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <vision_interface/robot_slots.h>

namespace tdt_radar {

// ── 盲区固定猜点 (来自 HKUST/Ultra 方案) ────────────────────────────────
// key: "R1".."R7", "B1".."B7"
using GuessPtList = std::vector<std::pair<float, float>>;
using GuessPtMap  = std::map<std::string, GuessPtList>;

// 机器人行为分类 (影响 PointGuesser 猜点偏好)
enum class RobotBehavior : uint8_t {
    NORMAL          = 0,  // 默认：全局猜点
    CHARGING        = 1,  // 检测到速度朝基地方向 → snap到充电区
    TUNNEL          = 2,  // 进入隧道入口 → snap到隧道内/出口
    OUTPOST_AMBUSH  = 3,  // 英雄消失在前哨站附近 → snap到前哨后吊射位
    ASSEMBLY        = 4,  // 工程消失在中央装配区方向 → snap到能量机关装配区
};

// 机器人运动能力分类 (全场持久追踪, 影响高台/飞坡/钻洞预测)
// 通过观察机器人进入特定区域得出，一旦确认不再降级
enum class MobilityHint : uint8_t {
    UNKNOWN  = 0,  // 未知: 尚未观测到特殊行为
    CRAWLER  = 1,  // 钻洞步兵: 进入隧道区 → 只能走隧道, 不能跳跃
    WHEELED  = 2,  // 轮腿/轮式: 能上斜坡但不能飞坡跳跃
    LEGGED   = 3,  // 腿足机器人 (串腿/并腿): 可飞坡/跳跃/上高台
};

struct GhostTrack {
    int    color;
    int    number;
    float  x, y;
    float  vx, vy;
    double timestamp;
    RobotBehavior behavior = RobotBehavior::NORMAL;
    MobilityHint  mobility = MobilityHint::UNKNOWN;
    // id_score snapshot for inheritance
    float  id_score[Kalman_filter_plus::N_IDS] = {};
};

class KalmanFilter : public rclcpp::Node {
public:
    KalmanFilter(const rclcpp::NodeOptions& node_options);
    ~KalmanFilter() {}

private:
    void normalizeDetectToGlobal(vision_interface::msg::DetectResult& msg);
    int resolveSelfColor() const;
    rcl_interfaces::msg::SetParametersResult on_parameters_changed(
        const std::vector<rclcpp::Parameter>& parameters);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Subscription<vision_interface::msg::DetectResult>::SharedPtr
        sub_detect_;
    // sub_lidar_ removed: /lidar_detect (dart/fly warn from dynamic_cloud) is
    // consumed only by debug_map for UI overlay; kalman_filter has no use for it.
    rclcpp::Subscription<vision_interface::msg::MatchInfo>::SharedPtr
        sub_match_;
    rclcpp::Publisher<vision_interface::msg::Radar2Sentry>::SharedPtr
        radar_pub_;
    rclcpp::Publisher<vision_interface::msg::DetectResult>::SharedPtr
        radar_detect_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void detect_callback(
        const vision_interface::msg::DetectResult::SharedPtr msg);
    void
    match_callback(const vision_interface::msg::MatchInfo::SharedPtr msg);
    std::vector<Kalman_filter_plus>  KFs;
    bool spatial_color_mode_ = false; // fallback: infer color from field-side when camera unavailable
    vision_interface::msg::MatchInfo match_info;

    float arena_width_ = 28.0F;
    float arena_height_ = 15.0F;
    bool input_is_self_frame_ = false;
    int  self_color_override_ = -1;

    float lidar_match_radius_ = 1.0F;
    float lidar_match_speed_gain_ = 2.5F;
    float track_timeout_s_ = 2.0F;
    int track_history_size_ = 20;
    float camera_time_threshold_s_ = 1.0F;
    float blind_timeout_s_ = 10.0F;
    bool  pos_reinforce_enable_ = false;
    // Intent-directed goal attraction (kalman_filter.cpp callback line ~514):
    // when camera silent > camera_time_threshold_s_, pull predict_point toward
    // pre-defined goal (base / supply / enemy half). Off by default to keep
    // minimap strictly showing observed positions, no predicted pulls.
    bool  goal_attraction_enable_ = false;
    // Engineer: supply zone at each base (X within depth from edge, Y in two side bands)
    float supply_x_depth_   = 5.5F;   // m from base edge
    float supply_y_band_lo_ = 2.5F;   // m low-side Y band start
    float supply_y_band_hi_ = 4.5F;   // m low-side Y band end (mirrored for top)
    int   engineer_reinforce_votes_ = 4;
    // Hero: near outpost (X=10.82m red / X=17.18m blue, CAD self_tower 锚点)
    float outpost_x_          = 10.82F;  // red outpost X (blue = arena_width - this)
    float outpost_zone_r_     = 2.5F;  // radius around outpost for hero hint
    int   hero_reinforce_votes_ = 2;
    bool  log_match_stats_      = false;
    float nms_merge_r_           = 0.8F;  // merge duplicate tracks closer than this (m)
    int   max_tracks_            = 20;    // hard cap; weakest tracks pruned first
    float static_filter_dist_    = 0.3F;  // max lifetime displacement to call a track static (m)
    float static_filter_age_s_   = 3.0F;  // min age before static filter applies (s)
    // --- ID scoring & KF noise params (passed through to Kalman_filter_plus::set_tuning) ---
    float id_score_inc_     = 40.0F;
    float id_score_dec_     =  2.0F;
    float id_score_decay_   =  5.0F;
    float id_score_thresh_  = 60.0F;
    float kf_cov_factor_    = 0.002F;
    float kf_cov_freeze_s_  = 3.0F;
    float ghost_inherit_r_  = 2.5F;   // radius to search ghost tracks for new-track ID seeding (m)
    float ghost_ttl_s_      = 30.0F;  // ghost track time-to-live (s) before expiry
    float id_score_hi_mult_ = 0.1F;   // decay multiplier when id_score is confirmed high (< 1 = slower decay)
    // ── HKUST-style 加权代价 (Algorithm 1 Step 4) ──────────────────────────
    //   cost = -(W1·HistConf + W3·BotIdMatch + W4·PosScore)
    //   PosScore = clamp(1 - d / max_field_dist, [0, 1])
    //   HistConf = id_score[(clr,num)] / id_score_thresh_ (clamped [0, 1])
    //   BotIdMatch = 1 if KF.last_bot_id == det.bot_id (both ≥ 0), else 0
    float cost_w_hist_   = 0.45F;
    float cost_w_botid_  = 0.15F;  // W3: short-term track ID bonus
    float cost_w_pos_    = 1.0F;
    float max_field_dist_ = 31.8F;   // sqrt(28² + 15²)
    std::vector<GhostTrack> ghost_tracks_;
    uint8_t prev_game_progress_ = 0;
    uint8_t prev_marks_[kRobotSlotCount] = {};   // 标记确认上一帧状态, 用于上升沿检测

    // ── 运动能力持久分类表 (全场不清空, 越来越精确) ───────────────────────
    // key = (color<<4)|number; value = MobilityHint
    std::map<uint8_t, MobilityHint> robot_mobility_;

    // ── 英雄前哨吊射检测参数 ────────────────────────────────────────────────
    float hero_outpost_zone_lo_ = 8.0F;   // hero消失的X下界 (红方视角: 8-16 = 前哨附近)
    float hero_outpost_zone_hi_ = 16.0F;  // hero消失的X上界 (对应蓝方: 28-hi ~ 28-lo)
    float hero_outpost_y_lo_    = 4.0F;   // hero消失的Y下界
    float hero_outpost_y_hi_    = 11.0F;  // hero消失的Y上界

    // ── 盲区猜点 (HKUST/Ultra PointGuesser) ──────────────────────────────
    GuessPtMap  guess_pts_;             // loaded from guess_pts_path param
    bool        guess_pts_enable_  = true;
    float       guess_d_factor_    = 0.08F;
    float       guess_cos_factor_  = 0.20F;
    float       guess_snap_max_    = 0.60F;  // max blend ratio toward best pt
    float       guess_snap_ramp_   = 0.15F;  // snap per second after start
    float       guess_snap_start_s_= 2.0F;   // seconds before snap begins

    // ── 雷达阴影图 (预计算盲区, 用于 PointGuesser 即时激活) ──────────────
    // config/outputs/shadow_red.bin / shadow_blue.bin
    // raw uint8, row-major, 1=可见 0=盲区, GRID_RES=0.05m, shape=(300,560)
    std::vector<uint8_t> shadow_grid_[2];  // [0]=red radar, [1]=blue radar
    static constexpr int   kShadowRows = 300;   // 15m / 0.05m
    static constexpr int   kShadowCols = 560;   // 28m / 0.05m
    static constexpr float kShadowRes  = 0.05F; // m per cell
    bool  shadow_enable_ = true;
    // 进入阴影区后立即开始snap (不等 snap_start_s), 且加速ramp
    float shadow_snap_ramp_mult_ = 2.0F;  // snap速率倍数 (阴影区内)
    // 活跃轨迹在阴影区内可延长超时 (不用等2s才ghost)
    float shadow_track_timeout_mult_ = 3.0F;  // track_timeout 延长倍数

    void  loadShadowGrid(const std::string& dir);
    bool  isBlindZone(float x, float y) const;

    // Helper: convert (color, slot) → robot key string e.g. "R1", "B7"
    static std::string robotKey(int color, int slot) {
        char buf[4];
        const int robot_no = slot_to_robot_number(slot);
        buf[0] = (color == 2) ? 'R' : 'B';
        buf[1] = static_cast<char>('0' + robot_no);
        buf[2] = '\0';
        return std::string(buf);
    }
    void load_guess_pts(const std::string& path);
    void pos_reinforce(Kalman_filter_plus& kf) const;
    void nms_tracks();
    void prune_static_tracks();
    void cap_track_count();

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
        params_callback_handle_;
};
}  // namespace tdt_radar
