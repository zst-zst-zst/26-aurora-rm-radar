#include "kalman_filter.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <pcl_conversions/pcl_conversions.h>
#include "filter_plus.h"
#include <yaml-cpp/yaml.h>

namespace {
// Hungarian algorithm (Jonker-Volgenant, O(n³))
// cost[i][j]: cost of assigning row i to col j (use kInf for forbidden pairs)
// Returns assign[i] = j (row i → col j), or -1 if unassigned.
static constexpr float kInf = 1e9F;
std::vector<int> hungarian(const std::vector<float>& cost, int n, int m)
{
    if (n == 0 || m == 0) return std::vector<int>(n, -1);
    const int sz = std::max(n, m);
    std::vector<float> u(sz + 1, 0.0F), v(sz + 1, 0.0F);
    std::vector<int>   p(sz + 1, 0), way(sz + 1, 0);
    std::vector<float> minv(sz + 1);  // pre-allocated, reset per outer iteration
    std::vector<bool>  used(sz + 1);  // pre-allocated, reset per outer iteration
    for (int i = 1; i <= sz; ++i) {
        p[0] = i;
        int j0 = 0;
        std::fill(minv.begin(), minv.end(), kInf);
        std::fill(used.begin(), used.end(), false);
        do {
            used[j0] = true;
            int   i0 = p[j0], j1 = -1;
            float delta = kInf;
            for (int j = 1; j <= sz; ++j) {
                if (!used[j]) {
                    float c = (i0 <= n && j <= m) ? cost[(i0 - 1) * m + (j - 1)] : kInf;
                    float cur = c - u[i0] - v[j];
                    if (cur < minv[j]) { minv[j] = cur; way[j] = j0; }
                    if (minv[j] < delta) { delta = minv[j]; j1 = j; }
                }
            }
            for (int j = 0; j <= sz; ++j) {
                if (used[j]) { u[p[j]] += delta; v[j] -= delta; }
                else minv[j] -= delta;
            }
            // BUG fix (heap corruption / double-free):
            // j1 stays at -1 when every j ∈ [1,sz] has been marked used in
            // the same outer iteration — possible with degenerate cost
            // matrices (all kInf, NaN propagation). Falling through with
            // j0 = -1 makes subsequent p[j0] read AND `p[j0] = p[j1]` write
            // out-of-bounds, silently corrupting adjacent heap allocations.
            // Bail out cleanly: leave the rest of this row unassigned.
            if (j1 < 0) break;
            j0 = j1;
        } while (p[j0] != 0);
        if (j0 < 0) continue;  // this row left unassigned by the bail-out above
        do { int j1 = way[j0]; p[j0] = p[j1]; j0 = j1; } while (j0);
    }
    std::vector<int> result(n, -1);
    for (int j = 1; j <= m; ++j) {
        if (p[j] > 0 && p[j] <= n && cost[(p[j] - 1) * m + (j - 1)] < kInf)
            result[p[j] - 1] = j - 1;
    }
    return result;
}
}  // namespace

namespace tdt_radar {

KalmanFilter::KalmanFilter(const rclcpp::NodeOptions& node_options)
    : rclcpp::Node("kalman_filter_node", node_options)
{
    arena_width_ = this->declare_parameter<float>("map_width", arena_width_);
    arena_height_ = this->declare_parameter<float>("map_height", arena_height_);
    input_is_self_frame_ =
        this->declare_parameter<bool>("input_is_self_frame", false);
    self_color_override_ = this->declare_parameter<int>("self_color_override", -1);
    lidar_match_radius_ =
        this->declare_parameter<float>("lidar_match_radius", lidar_match_radius_);
    lidar_match_speed_gain_ =
        this->declare_parameter<float>("lidar_match_speed_gain", lidar_match_speed_gain_);
    track_timeout_s_ =
        this->declare_parameter<float>("track_timeout_s", track_timeout_s_);
    track_history_size_ =
        this->declare_parameter<int>("track_history_size", track_history_size_);
    camera_time_threshold_s_ =
        this->declare_parameter<float>("camera_time_threshold_s", camera_time_threshold_s_);
    log_match_stats_ =
        this->declare_parameter<bool>("log_match_stats", log_match_stats_);
    spatial_color_mode_ =
        this->declare_parameter<bool>("spatial_color_mode", spatial_color_mode_);
    nms_merge_r_ =
        static_cast<float>(this->declare_parameter<double>("nms_merge_r", nms_merge_r_));
    // max_tracks: 比赛最多 14 机器人 (7×2队) + 噪声边界, 直接固化为 20。
    static_filter_dist_ =
        static_cast<float>(this->declare_parameter<double>("static_filter_dist", static_filter_dist_));
    static_filter_age_s_ =
        static_cast<float>(this->declare_parameter<double>("static_filter_age_s", static_filter_age_s_));
    blind_timeout_s_ =
        static_cast<float>(this->declare_parameter<double>("blind_timeout_s", 10.0));
    pos_reinforce_enable_ = this->declare_parameter<bool>("pos_reinforce_enable", false);
    goal_attraction_enable_ = this->declare_parameter<bool>("goal_attraction_enable", false);
    id_score_inc_    = static_cast<float>(this->declare_parameter<double>("id_score_inc",    40.0));
    id_score_dec_    = static_cast<float>(this->declare_parameter<double>("id_score_dec",     2.0));
    id_score_decay_  = static_cast<float>(this->declare_parameter<double>("id_score_decay",   5.0));
    id_score_thresh_ = static_cast<float>(this->declare_parameter<double>("id_score_thresh", 60.0));
    kf_cov_factor_   = static_cast<float>(this->declare_parameter<double>("kf_cov_factor",  0.002));
    kf_cov_freeze_s_ = static_cast<float>(this->declare_parameter<double>("kf_cov_freeze_s", 3.0));
    id_score_hi_mult_ = static_cast<float>(this->declare_parameter<double>("id_score_hi_mult", 0.1));
    // HKUST 风格加权代价 (HistConf + BotIdMatch + PosScore)
    cost_w_hist_      = static_cast<float>(this->declare_parameter<double>("cost_w_hist",  0.45));
    cost_w_botid_     = static_cast<float>(this->declare_parameter<double>("cost_w_botid", 0.15));
    cost_w_pos_       = static_cast<float>(this->declare_parameter<double>("cost_w_pos",   1.0));
    ghost_inherit_r_  = static_cast<float>(this->declare_parameter<double>("ghost_inherit_r",  2.5));
    ghost_ttl_s_      = static_cast<float>(this->declare_parameter<double>("ghost_ttl_s",     30.0));

    // ── CAD / 规则常量 (RM2026 V1.4.2 + 场地 V1.2.0, 锁死, 不可调) ────────
    //   max_field_dist : sqrt(W² + H²) 场地对角
    //   supply_*       : 补给区 footprint (己方基地)
    //   outpost_x      : 红方前哨 X 坐标 (蓝方 = arena_w - outpost_x)
    //                    取 calibrate_points_red.yaml::self_tower X = 10.82m (CAD 实测)
    //   outpost_zone_r : 前哨警戒圈半径
    //   *_reinforce_votes : ID 加固投票阈值 (固定为论文实测最优)
    max_field_dist_         = std::sqrt(arena_width_ * arena_width_ +
                                        arena_height_ * arena_height_);
    supply_x_depth_         = 5.5F;
    supply_y_band_lo_       = 2.5F;
    supply_y_band_hi_       = 4.5F;
    outpost_x_              = 10.82F;  // CAD self_tower 锚点
    outpost_zone_r_         = 2.5F;
    engineer_reinforce_votes_ = 4;
    hero_reinforce_votes_     = 2;

    // ── 盲区猜点表加载 (HKUST/Ultra PointGuesser) ──────────────────────
    const auto guess_pts_path = this->declare_parameter<std::string>(
        "guess_pts_path", "config/guess_pts.yaml");
    guess_pts_enable_ = this->declare_parameter<bool>("guess_pts_enable", true);
    guess_d_factor_   = static_cast<float>(
        this->declare_parameter<double>("guess_d_factor", 0.08));
    guess_cos_factor_ = static_cast<float>(
        this->declare_parameter<double>("guess_cos_factor", 0.20));
    guess_snap_max_   = static_cast<float>(
        this->declare_parameter<double>("guess_snap_max", 0.60));
    guess_snap_ramp_  = static_cast<float>(
        this->declare_parameter<double>("guess_snap_ramp", 0.15));
    guess_snap_start_s_ = static_cast<float>(
        this->declare_parameter<double>("guess_snap_start_s", 2.0));
    if (guess_pts_enable_) {
        load_guess_pts(guess_pts_path);
        RCLCPP_INFO(this->get_logger(),
            "Guess pts loaded: %zu robot types from %s",
            guess_pts_.size(), guess_pts_path.c_str());
    }
    shadow_enable_ = this->declare_parameter<bool>("shadow_enable", true);
    if (shadow_enable_) {
        const auto shadow_dir = this->declare_parameter<std::string>(
            "shadow_dir", "config/outputs");
        loadShadowGrid(shadow_dir);
    }

    params_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&KalmanFilter::on_parameters_changed, this,
                  std::placeholders::_1));

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox/lidar_cluster", 10,
        std::bind(&KalmanFilter::callback, this, std::placeholders::_1));
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/livox/lidar_kalman", 10);
    radar_pub_ =
        this->create_publisher<vision_interface::msg::Radar2Sentry>(
            "/radar2sentry", 10);
    radar_detect_pub_ =
        this->create_publisher<vision_interface::msg::DetectResult>(
            "/kalman_detect", 10);
    const bool enable_aux_input =
        this->declare_parameter<bool>("enable_aux_input", true);
    const bool disable_aux_input =
        std::getenv("TDT_DISABLE_KALMAN_AUX_INPUT") != nullptr;
    if (enable_aux_input && !disable_aux_input) {
        sub_detect_ =
            this->create_subscription<vision_interface::msg::DetectResult>(
                "/resolve_result", rclcpp::SensorDataQoS(),
                std::bind(&KalmanFilter::detect_callback, this,
                          std::placeholders::_1));
        sub_match_ =
            this->create_subscription<vision_interface::msg::MatchInfo>(
                "/match_info", 10,
                std::bind(&KalmanFilter::match_callback, this,
                          std::placeholders::_1));
    } else {
        RCLCPP_WARN(this->get_logger(),
                    "Kalman aux inputs disabled (%s%s)",
                    enable_aux_input ? "" : "parameter",
                    disable_aux_input ? " env" : "");
    }

    RCLCPP_INFO(this->get_logger(), "Kalman_filter_Node has been started.");
    RCLCPP_INFO(this->get_logger(),
                "Kalman config: map_width=%.2f map_height=%.2f input_is_self_frame=%d self_color_override=%d",
                arena_width_, arena_height_, input_is_self_frame_ ? 1 : 0,
                self_color_override_);
    RCLCPP_INFO(this->get_logger(),
                "Kalman prediction policy: camera_hit=hard_reset pos_reinforce=%d goal_attraction=%d guess_pts=%d shadow=%d",
                pos_reinforce_enable_ ? 1 : 0,
                goal_attraction_enable_ ? 1 : 0,
                guess_pts_enable_ ? 1 : 0,
                shadow_enable_ ? 1 : 0);

}

int KalmanFilter::resolveSelfColor() const
{
    if (self_color_override_ == 0 || self_color_override_ == 2) {
        return self_color_override_;
    }
    if (match_info.self_color == 0 || match_info.self_color == 2) {
        return match_info.self_color;
    }
    return -1;
}

void KalmanFilter::normalizeDetectToGlobal(
    vision_interface::msg::DetectResult& msg)
{
    if (!input_is_self_frame_) {
        return;
    }

    const int self_color = resolveSelfColor();
    if (self_color != 0 && self_color != 2) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "input_is_self_frame=true but self_color is unknown, skip normalization");
        return;
    }

    // If the input is already in red-global frame (self red), no transform is needed.
    if (self_color == 2) {
        return;
    }

    // Blue-self view -> global referee frame by 180-degree rotation around map center.
    for (int i = 0; i < kRobotSlotCount; i++) {
        if (msg.blue_x[i] != 0.0F && msg.blue_y[i] != 0.0F) {
            msg.blue_x[i] = arena_width_ - msg.blue_x[i];
            msg.blue_y[i] = arena_height_ - msg.blue_y[i];
        }
        if (msg.red_x[i] != 0.0F && msg.red_y[i] != 0.0F) {
            msg.red_x[i] = arena_width_ - msg.red_x[i];
            msg.red_y[i] = arena_height_ - msg.red_y[i];
        }
    }
}

void KalmanFilter::match_callback(
    const vision_interface::msg::MatchInfo::SharedPtr msg)
{
    const uint8_t new_prog  = msg->game_progress;
    const uint8_t prev_prog = prev_game_progress_;
    this->match_info     = *msg;
    prev_game_progress_  = new_prog;

    // ── 标记确认正反馈: 裁判系统确认标记成功 → 锁定 ID 置信度 ──────────────
    // marks[i] 上升沿 (0→非零) = 我方雷达成功标记了敌方机器人 i+1
    // 意义: 这是唯一来自裁判系统的身份真值 → 对当前最高置信 KF 大幅加分,
    //       并抹除其他 KF 对同一 ID 的竞争, 防止标记成功后 ID 互换
    const int self_c_mark = resolveSelfColor();
    const bool mark_active = (self_c_mark >= 0 && match_info.match_time > 0);
    for (int i = 0; i < kRobotSlotCount; ++i) {
        const uint8_t cur  = msg->marks[i];
        const uint8_t prev = prev_marks_[i];
        // 上升沿正反馈仅在比赛进行中且 self_color 已知时才触发,
        // 但 prev_marks_ 每帧都要刷新, 否则赛前/赛后残留状态会在重启后误触发。
        if (mark_active && cur != 0 && prev == 0) {
            const int slot = (self_c_mark == 2) ? i : (kRobotSlotCount + i);
            // 找分数最高的 KF (最可能就是被标记的那辆)
            int   best_kf  = -1;
            float best_sc  = 0.0F;
            for (int k = 0; k < (int)KFs.size(); ++k) {
                if (KFs[k].id_score[slot] > best_sc) {
                    best_sc = KFs[k].id_score[slot]; best_kf = k;
                }
            }
            if (best_kf >= 0) {
                // 正反馈: 加 10× inc (相当于约10帧的确认量)
                KFs[best_kf].id_score[slot] = std::min(
                    id_score_inc_ * 25.0F,
                    KFs[best_kf].id_score[slot] + id_score_inc_ * 10.0F);
                // HKUST state machine: 裁判系统确认 = 最强证据 → 重置为 CONFIRMED
                KFs[best_kf].hit_count_  = std::max(KFs[best_kf].hit_count_,
                                                    Kalman_filter_plus::HIT_THRESHOLD + 3);
                KFs[best_kf].miss_count_ = 0;
                // 竞争抑制: 其余 KF 对同一 ID 减半, 防止 ID 互换
                for (int k = 0; k < (int)KFs.size(); ++k) {
                    if (k != best_kf) KFs[k].id_score[slot] *= 0.3F;
                }
                RCLCPP_INFO(get_logger(),
                    "Mark confirmed: enemy slot=%d (marks[%d]↑) → KF[%d] id_score+=%.0f",
                    slot, i, best_kf, id_score_inc_ * 10.0F);
            }
        }
        prev_marks_[i] = cur;
    }

    // game_progress: 1=准备阶段(有人在场), 2=自检(场地已清空,比赛前~15s)
    // 当 1→2 时场地刚刚清场，立即清空所有轨迹，避免志愿者/工作人员的误检污染比赛数据。
    if (prev_prog == 1U && new_prog >= 2U) {
        const size_t n = KFs.size();
        KFs.clear();
        ghost_tracks_.clear();
        RCLCPP_INFO(this->get_logger(),
                    "Field cleared (game_progress %u→%u): purged %zu prep-phase tracks.",
                    static_cast<unsigned>(prev_prog), static_cast<unsigned>(new_prog),
                    n);
    }

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                          "MatchInfo updated: self_color=%d match_time=%d game_progress=%u",
                          static_cast<int>(msg->self_color),
                          static_cast<int>(msg->match_time),
                          static_cast<unsigned>(new_prog));
}

void KalmanFilter::detect_callback(
    const vision_interface::msg::DetectResult::SharedPtr msg)
{
    vision_interface::msg::DetectResult normalized_msg = *msg;
    normalizeDetectToGlobal(normalized_msg);

    rclcpp::Time time       = msg->header.stamp;
    const double input_time = Kalman_filter_plus::GetTimeByRosTime(time);
    const int    n_kfs      = static_cast<int>(KFs.size());

    // Camera-primary: pre-compute each KF's interpolated position at the camera timestamp.
    // KF histories are stamped; pick the entry closest in time within threshold.
    std::vector<pcl::PointXY> kf_hist(n_kfs);
    std::vector<bool>         kf_ok(n_kfs, false);
    for (int i = 0; i < n_kfs; ++i) {
        double best_dt = KFs[i].camera_time_threshold_;
        for (const auto& h : KFs[i].history) {
            double dt = std::abs(h.first - input_time);
            if (dt < best_dt) { best_dt = dt; kf_hist[i] = h.second; kf_ok[i] = true; }
        }
        // Fallback: if LiDAR history is stale but track is alive (e.g. LiDAR blind spot),
        // use the current Kalman prediction so camera can still keep the track alive.
        if (!kf_ok[i] && KFs[i].has_updated) {
            kf_hist[i] = KFs[i].predict_point;
            kf_ok[i]   = true;
        }
    }

    // For each color, apply Hungarian exclusive assignment:
    //   one camera detection  → at most one KF
    //   one KF                → at most one detection
    bool any_new_track = false;
    for (int clr : {0, 2}) {
        // Collect valid detections {position, robot_number, bot_id}
        struct Det { pcl::PointXY pos; int number; int bot_id; };
        std::vector<Det> dets;
        dets.reserve(kRobotSlotCount);
        for (int i = 0; i < kRobotSlotCount; i++) {
            const float x = (clr == 2) ? normalized_msg.red_x[i]  : normalized_msg.blue_x[i];
            const float y = (clr == 2) ? normalized_msg.red_y[i]  : normalized_msg.blue_y[i];
            if (x == 0.0F && y == 0.0F) continue;
            pcl::PointXY pt; pt.x = x; pt.y = y;
            const int bid = (clr == 2) ? normalized_msg.red_bot_id[i]
                                       : normalized_msg.blue_bot_id[i];
            dets.push_back({pt, i, bid});
        }
        if (dets.empty()) continue;

        const int n_det = static_cast<int>(dets.size());
        std::vector<bool> det_matched(n_det, false);

        if (n_kfs > 0) {
            // Build cost matrix [n_kfs × n_det] (flat row-major for cache efficiency)
            // HKUST Algorithm 1 Step 4 风格加权代价:
            //   PosScore = clamp(1 - d/max_field_dist, [0,1])
            //   HistConf = KF 对 (clr, num) 的累积 id_score 归一化
            //   BotIdMatch = 1 if KF.last_bot_id == det.bot_id (both ≥ 0)
            //   cost = -(W1·HistConf + W3·BotIdMatch + W4·PosScore)
            std::vector<float> cost(n_kfs * n_det, kInf);
            const float max_d = (max_field_dist_ > 1e-3F) ? max_field_dist_ : 31.8F;
            for (int i = 0; i < n_kfs; ++i) {
                if (!kf_ok[i]) continue;
                // Color-consistency gate: skip KFs whose confirmed color differs from clr.
                // Prevents a confirmed-red track from stealing a blue camera detection.
                // Unknown color (1) is allowed through either loop.
                const int kf_clr = KFs[i].get_color();
                if (kf_clr != 1 && kf_clr != clr) continue;
                for (int j = 0; j < n_det; ++j) {
                    const float d = KFs[i].Distance(kf_hist[i], dets[j].pos);
                    if (d >= KFs[i].detect_r) continue;  // distance gate
                    float pos_score = 1.0F - d / max_d;
                    if (pos_score < 0.0F) pos_score = 0.0F;
                    float hist_conf = KFs[i].get_hist_conf(clr, dets[j].number);
                    if (hist_conf > 1.0F) hist_conf = 1.0F;
                    // BotIdMatch: 1 if this KF last matched the same detect track
                    const float bot_match = (KFs[i].last_bot_id_ >= 0 && dets[j].bot_id >= 0 &&
                                             KFs[i].last_bot_id_ == dets[j].bot_id)
                                            ? 1.0F : 0.0F;
                    const float s = cost_w_hist_ * hist_conf
                                  + cost_w_botid_ * bot_match
                                  + cost_w_pos_ * pos_score;
                    cost[i * n_det + j] = -s;
                }
            }

            // Hungarian: each KF gets at most one detection, each detection at most one KF
            auto assign = hungarian(cost, n_kfs, n_det);
            for (int i = 0; i < n_kfs; ++i) {
                if (assign[i] < 0) {
                    // HKUST state machine: unmatched confirmed track → increment miss_count
                    if (KFs[i].hit_count_ >= Kalman_filter_plus::HIT_THRESHOLD) {
                        KFs[i].miss_count_++;
                    }
                    continue;
                }
                const int j = assign[i];
                det_matched[j] = true;
                KFs[i].update_id_score(clr, dets[j].number);
                KFs[i].detect_history.push_back({clr, dets[j].number});
                if ((int)KFs[i].detect_history.size() > KFs[i].max_history)
                    KFs[i].detect_history.erase(KFs[i].detect_history.begin());
                // HKUST state machine: matched → increment hit_count, reset miss_count
                KFs[i].hit_count_++;
                KFs[i].miss_count_ = 0;
                // Update short-term track ID for next frame’s W3 cost
                KFs[i].last_bot_id_ = dets[j].bot_id;
                // Camera-primary: if armor is visible, the resolved camera
                // position is the current truth.  Prediction/guessing is only
                // allowed after the robot disappears into the lost/ghost path.
                KFs[i].confirm_count_ = std::min(KFs[i].confirm_count_ + 1, 5);
                KFs[i].update_camera(dets[j].pos, time);
            }
        }

        // Camera-primary: spawn new KF tracks for unmatched camera detections.
        // Camera already provides robot identity (color + number), so confirm immediately.
        for (int j = 0; j < n_det; ++j) {
            if (det_matched[j]) continue;
            KFs.emplace_back(dets[j].pos, time);
            auto& kf = KFs.back();
            kf.set_tuning(lidar_match_radius_, lidar_match_speed_gain_,
                          track_history_size_, camera_time_threshold_s_,
                          id_score_inc_, id_score_dec_, id_score_decay_,
                          id_score_thresh_, kf_cov_factor_, kf_cov_freeze_s_,
                          id_score_hi_mult_);
            kf.update_id_score(clr, dets[j].number);
            kf.detect_history.push_back({clr, dets[j].number});
            kf.last_bot_id_ = dets[j].bot_id;
            kf.camera_last_time = 0.0F;
            kf.last_time        = 0.0F;
            kf.confirm_count_   = 2;  // camera identity is reliable; skip tentative stage
            kf.hit_count_       = 2;  // camera identity is reliable; start as CONFIRMED
            kf.miss_count_      = 0;
            // Ghost ID inheritance (same velocity-direction scoring as LiDAR new-track path)
            if (!ghost_tracks_.empty()) {
                float best_score = -1e9F;
                int   best_g     = -1;
                const float px = dets[j].pos.x, py = dets[j].pos.y;
                constexpr float COS_W   = 0.35F;
                constexpr float DIST_W  = 0.65F;
                constexpr float D_FACTOR = 0.5F;
                for (int g = 0; g < (int)ghost_tracks_.size(); ++g) {
                    const float dx   = px - ghost_tracks_[g].x;
                    const float dy   = py - ghost_tracks_[g].y;
                    const float dist = std::sqrt(dx * dx + dy * dy);
                    if (dist > ghost_inherit_r_) continue;
                    const float d_score = std::exp(-dist * D_FACTOR);
                    const float vx    = ghost_tracks_[g].vx;
                    const float vy    = ghost_tracks_[g].vy;
                    const float v_len = std::sqrt(vx * vx + vy * vy);
                    float cos_sim = 0.0F;
                    if (v_len > 0.1F && dist > 0.01F)
                        cos_sim = (vx * dx + vy * dy) / (v_len * dist);
                    const float score = COS_W * cos_sim + DIST_W * d_score;
                    if (score > best_score) { best_score = score; best_g = g; }
                }
                    if (best_g >= 0) {
                        for (int s = 0; s < Kalman_filter_plus::N_IDS; ++s)
                            kf.id_score[s] = ghost_tracks_[best_g].id_score[s] * 0.8F;
                        ghost_tracks_.erase(ghost_tracks_.begin() + best_g);
                    }
                }
            any_new_track = true;
        }
    }
    // Merge any duplicates spawned by camera this cycle
    if (any_new_track) nms_tracks();
}

void KalmanFilter::callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    rclcpp::Time time = msg->header.stamp;
    auto         now_time = std::chrono::steady_clock::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXY>::Ptr cloud_xy(
        new pcl::PointCloud<pcl::PointXY>);
    pcl::fromROSMsg(*msg, *cloud);
    cloud_xy->points.reserve(cloud->points.size());
    for (const auto& point : cloud->points) {
        pcl::PointXY point_xy;
        point_xy.x = point.x;
        point_xy.y = point.y;
        cloud_xy->points.push_back(point_xy);
    }
    for (auto& kf : KFs) {
        kf.update_predict_point(Kalman_filter_plus::GetTimeByRosTime(time));
        kf.has_updated = false;

        // Intent-directed goal attraction for long occlusions (>1s).
        // Human-brain logic: each robot type has a likely destination based on game state.
        // Apply weak attraction force that grows with occlusion duration.
        // Strength: 0 at 0.3s, ramps to max 0.3 m/s² equivalent at 3s.
        //
        // Disable via launch param goal_attraction_enable=false to keep minimap
        // strictly showing actually-observed positions (no "predicted goal" pulls).
        if (goal_attraction_enable_ && kf.camera_last_time > camera_time_threshold_s_) {
            int color = 1, number = -1;
            kf.get_id(color, number);              // slot 0=hero 1=eng 2/3=inf 4=sentry; -1 = unknown
            const float t_occ  = kf.camera_last_time;
            const float alpha  = std::min(0.08F, 0.015F * (t_occ - camera_time_threshold_s_));  // max 8% pull/step

            // Goal positions in referee frame (red-origin).
            // When color==2 (red), self base is at X~2, enemy at X~26.
            // When color==0 (blue), self base is at X~26, enemy at X~2.
            float goal_x = kf.predict_point.x;  // default: no goal, stay put
            float goal_y = kf.predict_point.y;

            const bool is_red  = (color == 2);
            const bool is_blue = (color == 0);
            const float mid_y  = arena_height_ * 0.5F;  // 7.5m
            const float enemy_base_x  = is_red  ? (arena_width_ - 3.0F) :  3.0F;
            const float self_base_x   = is_red  ?  3.0F : (arena_width_ - 3.0F);
            const float supply_y_lo   = supply_y_band_lo_;
            const float supply_y_hi   = supply_y_band_hi_;
            const float self_supply_x  = is_red  ? supply_x_depth_ * 0.5F
                                                  : (arena_width_ - supply_x_depth_ * 0.5F);

            // V1.3.0 0x0003 仅提供己方 HP: robot_hp[0..7] = self_color 一侧。
            // 敌方 track 没有 HP 可读, 视作满血 (600), 等价于关闭撤退逻辑。
            const int hp_idx = slot_to_hp_index(number);
            const int self_c_attr = resolveSelfColor();
            const bool track_is_self = (color == self_c_attr);
            const uint16_t own_hp = (track_is_self && hp_idx >= 0)
                ? match_info.robot_hp[hp_idx]
                : 600U;

            if (number == kHeroSlot && (is_red || is_blue)) {
                goal_x = (own_hp < 100) ? self_base_x : enemy_base_x;
                goal_y = mid_y;
            } else if (number == 1 && (is_red || is_blue)) {
                goal_x = self_supply_x;
                goal_y = (supply_y_lo + supply_y_hi) * 0.5F;
            } else if (number == 2 || number == 3) {
                const float enemy_half_x = is_red ? arena_width_ * 0.7F : arena_width_ * 0.3F;
                goal_x = (own_hp < 80) ? self_base_x : enemy_half_x;
                goal_y = kf.predict_point.y;
            } else if (number == kSentrySlot) {
                goal_x = kf.predict_point.x;
                goal_y = mid_y;
            }

            // Apply fractional pull toward goal
            kf.predict_point.x += alpha * (goal_x - kf.predict_point.x);
            kf.predict_point.y += alpha * (goal_y - kf.predict_point.y);
        }

        // Map-constraint: eject predicted point from solid obstacle cylinders.
        // RM2026 obstacles (referee frame, red-origin): two 1.87m towers + center highland.
        // Only applied when the track has been unobserved (camera_last_time > 0.3s).
        if (kf.camera_last_time > 0.3F) {
            struct Obs { float cx, cy, r; };
            // CAD-derived obstacle footprints (referee frame):
            //   Towers @ (10.82, 3.66) & (17.18, 11.34): blob 0.46×0.57m -> r=0.45 (含余量)
            //   Center Highland @ (14, 7.5): CAD blob 2.6×2.6m, 内切圆 r=1.30, 外接圆 r=1.84
            //                                取 r=1.50 (1.3 + 0.2 margin), 避免过度弹出
            static const Obs OBSTACLES[] = {
                {10.82F, 3.66F,  0.45F},   // Self Tower  (CAD footprint 0.46×0.57m)
                {17.18F, 11.34F, 0.45F},   // Cross Tower (镜像)
                {14.00F,  7.50F, 1.50F},   // Center Highland (CAD 2.6×2.6m → 内切 + margin)
            };
            for (const auto& obs : OBSTACLES) {
                const float dx = kf.predict_point.x - obs.cx;
                const float dy = kf.predict_point.y - obs.cy;
                const float d2 = dx * dx + dy * dy;
                if (d2 < obs.r * obs.r && d2 > 1e-8F) {  // sqrt only when actually inside
                    const float inv_d = 1.0F / std::sqrt(d2);
                    kf.predict_point.x = obs.cx + dx * inv_d * obs.r;
                    kf.predict_point.y = obs.cy + dy * inv_d * obs.r;
                }
            }
        }
    }

    size_t lidar_new_tracks = 0;
    size_t lidar_updated_tracks = 0;

    const int n_kfs = static_cast<int>(KFs.size());
    const int n_pts = static_cast<int>(cloud_xy->points.size());
    if (n_kfs > 0 && n_pts > 0) {
        // Cascade matching (ByteTrack/DeepSORT style):
        // Pass 1 — confirmed tracks (confirm_count>=2) bid on ALL points first.
        // Pass 2 — tentative tracks bid only on points left unmatched by pass 1.
        // Prevents a noise spike from stealing a confirmed track's update.
        std::vector<int> conf_idx, tent_idx;
        conf_idx.reserve(n_kfs); tent_idx.reserve(n_kfs);
        for (int i = 0; i < n_kfs; ++i) {
            (KFs[i].confirm_count_ >= 2 ? conf_idx : tent_idx).push_back(i);
        }

        std::vector<bool> point_matched(n_pts, false);

        // ── Pass 1: confirmed tracks ──────────────────────────────────────
        if (!conf_idx.empty()) {
            const int nc = static_cast<int>(conf_idx.size());
            std::vector<float> cost(nc * n_pts, kInf);
            for (int ci = 0; ci < nc; ++ci)
                for (int j = 0; j < n_pts; ++j) {
                    const float c = KFs[conf_idx[ci]].match_and_cost(cloud_xy->points[j]);
                    if (c >= 0.0F) cost[ci * n_pts + j] = c;
                }
            auto asgn = hungarian(cost, nc, n_pts);
            for (int ci = 0; ci < nc; ++ci) {
                const int i = conf_idx[ci];
                if (asgn[ci] >= 0) {
                    KFs[i].update(cloud_xy->points[asgn[ci]], time);
                    point_matched[asgn[ci]] = true;
                    KFs[i].confirm_count_ = std::min(KFs[i].confirm_count_ + 1, 5);
                    // HKUST state machine: LiDAR hit → increment hit_count, reset miss_count
                    KFs[i].hit_count_++;
                    KFs[i].miss_count_ = 0;
                    lidar_updated_tracks++;
                } else {
                    // HKUST state machine: unmatched confirmed track → increment miss_count
                    if (KFs[i].hit_count_ >= Kalman_filter_plus::HIT_THRESHOLD) {
                        KFs[i].miss_count_++;
                    }
                    // Camera-primary: don't penalize confirm_count when camera is actively tracking
                    if (KFs[i].camera_last_time >= camera_time_threshold_s_)
                        KFs[i].confirm_count_ = std::max(0, KFs[i].confirm_count_ - 1);
                }
            }
        }

        // ── Pass 2: tentative tracks vs remaining points ──────────────────
        if (!tent_idx.empty()) {
            std::vector<int> rem;
            rem.reserve(n_pts);
            for (int j = 0; j < n_pts; ++j)
                if (!point_matched[j]) rem.push_back(j);

            if (!rem.empty()) {
                const int nt = static_cast<int>(tent_idx.size());
                const int nr = static_cast<int>(rem.size());
                std::vector<float> cost(nt * nr, kInf);
                for (int ti = 0; ti < nt; ++ti)
                    for (int rj = 0; rj < nr; ++rj) {
                        const float c = KFs[tent_idx[ti]].match_and_cost(
                            cloud_xy->points[rem[rj]]);
                        if (c >= 0.0F) cost[ti * nr + rj] = c;
                    }
                auto asgn = hungarian(cost, nt, nr);
                for (int ti = 0; ti < nt; ++ti) {
                    const int i = tent_idx[ti];
                    if (asgn[ti] >= 0) {
                        KFs[i].update(cloud_xy->points[rem[asgn[ti]]], time);
                        point_matched[rem[asgn[ti]]] = true;
                        KFs[i].confirm_count_ = std::min(KFs[i].confirm_count_ + 1, 5);
                        // HKUST state machine: LiDAR hit → increment hit_count, reset miss_count
                        KFs[i].hit_count_++;
                        KFs[i].miss_count_ = 0;
                        lidar_updated_tracks++;
                    } else {
                        // HKUST state machine: unmatched tentative track → increment miss_count
                        if (KFs[i].hit_count_ > 0) {
                            KFs[i].miss_count_++;
                        }
                        if (KFs[i].camera_last_time >= camera_time_threshold_s_)
                            KFs[i].confirm_count_ = std::max(0, KFs[i].confirm_count_ - 1);
                    }
                }
            } else {
                for (const int i : tent_idx)
                    if (KFs[i].camera_last_time >= camera_time_threshold_s_)
                        KFs[i].confirm_count_ = std::max(0, KFs[i].confirm_count_ - 1);
            }
        }

        // ── New tracks from still-unmatched points ────────────────────────
        for (int j = 0; j < n_pts; ++j) {
            if (!point_matched[j]) {
                KFs.emplace_back(cloud_xy->points[j], time);
                auto& kf = KFs.back();
                kf.set_tuning(lidar_match_radius_, lidar_match_speed_gain_,
                              track_history_size_, camera_time_threshold_s_,
                              id_score_inc_, id_score_dec_, id_score_decay_,
                              id_score_thresh_, kf_cov_factor_, kf_cov_freeze_s_,
                              id_score_hi_mult_);
                kf.hit_count_ = 1;  // LiDAR-spawned track starts as TENTATIVE
                kf.miss_count_ = 0;
                // Ghost track ID inheritance with velocity-direction weighting
                // (PointGuesser technique: score = cos_w*cos_sim + dist_w*exp(-d*d_factor))
                // Ghosts moving toward the new point score higher than ghosts that happened
                // to be nearby but pointing away — reduces cross-track ID contamination.
                if (!ghost_tracks_.empty()) {
                    float best_score = -1e9F;
                    int   best_g     = -1;
                    const float px = cloud_xy->points[j].x;
                    const float py = cloud_xy->points[j].y;
                    constexpr float COS_W  = 0.35F;  // weight for velocity alignment
                    constexpr float DIST_W = 0.65F;  // weight for proximity
                    constexpr float D_FACTOR = 0.5F; // exp decay rate (m^-1)
                    for (int g = 0; g < (int)ghost_tracks_.size(); ++g) {
                        const float dx   = px - ghost_tracks_[g].x;
                        const float dy   = py - ghost_tracks_[g].y;
                        const float dist = std::sqrt(dx * dx + dy * dy);
                        if (dist > ghost_inherit_r_) continue; // hard radius gate
                        // Distance score: exponential decay
                        const float d_score = std::exp(-dist * D_FACTOR);
                        // Velocity alignment (cos similarity)
                        const float vx    = ghost_tracks_[g].vx;
                        const float vy    = ghost_tracks_[g].vy;
                        const float v_len = std::sqrt(vx * vx + vy * vy);
                        float cos_sim = 0.0F;
                        if (v_len > 0.1F && dist > 0.01F) {
                            cos_sim = (vx * dx + vy * dy) / (v_len * dist);
                        }
                        const float score = COS_W * cos_sim + DIST_W * d_score;
                        if (score > best_score) { best_score = score; best_g = g; }
                    }
                    if (best_g >= 0) {
                        for (int s = 0; s < Kalman_filter_plus::N_IDS; ++s)
                            kf.id_score[s] = ghost_tracks_[best_g].id_score[s] * 0.8F;
                        ghost_tracks_.erase(ghost_tracks_.begin() + best_g);
                    }
                }
                lidar_new_tracks++;
            }
        }
        nms_tracks();
        prune_static_tracks();
        cap_track_count();
    } else {
        for (auto& point : cloud_xy->points) {
            KFs.emplace_back(point, time);
            auto& kf = KFs.back();
            kf.set_tuning(lidar_match_radius_, lidar_match_speed_gain_,
                          track_history_size_, camera_time_threshold_s_,
                          id_score_inc_, id_score_dec_, id_score_decay_,
                          id_score_thresh_, kf_cov_factor_, kf_cov_freeze_s_,
                          id_score_hi_mult_);
            kf.hit_count_ = 1;  // LiDAR-spawned track starts as TENTATIVE
            kf.miss_count_ = 0;
            lidar_new_tracks++;
        }
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    const double now_sec = std::chrono::duration_cast<std::chrono::duration<double>>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    for (int i = KFs.size() - 1; i >= 0; i--) {
        // Use the more recent of LiDAR or camera keep-alive for timeout decision
        const float effective_age = std::min(KFs[i].last_time, KFs[i].camera_last_time);
        // 阴影区内延长超时: 激光看不到是预期的, 不应立即ghost化
        const float eff_timeout = (shadow_enable_ &&
            isBlindZone(KFs[i].predict_point.x, KFs[i].predict_point.y))
            ? track_timeout_s_ * shadow_track_timeout_mult_
            : track_timeout_s_;
        if (effective_age > eff_timeout) {
            if (KFs[i].has_id()) {
                GhostTrack ghost;
                // 用 id_score (HKUST 风格) 推导, 与 publish 路径一致;
                // 之前用 detect_history 计票 (get_color/get_number) 在平票时
                // 会输出 (color=1, number=-1) → mob_key 溢出 + ghost map collapse。
                int gc = -1, gn = -1;
                KFs[i].get_id(gc, gn);
                if (gn < 0 || (gc != 0 && gc != 2)) {
                    // id_score 阈值已过但 get_id 仍判 unknown: 极端罕见, 跳过 ghost 化。
                    KFs.erase(KFs.begin() + i);
                    continue;
                }
                ghost.color  = gc;
                ghost.number = gn;
                ghost.x      = KFs[i].predict_point.x;
                ghost.y      = KFs[i].predict_point.y;
                ghost.vx     = KFs[i].get_vx();
                ghost.vy     = KFs[i].get_vy();
                ghost.timestamp = now_sec;
                // ── 行为分类 + 运动能力更新 ───────────────────────────────
                {
                    const float gx = ghost.x, gy = ghost.y;
                    const float vx = ghost.vx, vy = ghost.vy;
                    const bool near_blue_base = (gx > arena_width_ - supply_x_depth_ - 2.0F);
                    const bool near_red_base  = (gx < supply_x_depth_ + 2.0F);
                    const bool heading_blue   = (vx > 0.8F);
                    const bool heading_red    = (vx < -0.8F);
                    // TUNNEL: 实际隧道位置 (来自 simulate_shadow.py 标定)
                    // 红方: x=8-12, y=3-4.2 和 y=10.8-12
                    // 蓝方: x=16-20, y=3-4.2 和 y=10.8-12
                    const bool in_tunnel_zone =
                        ((gx > 7.5F && gx < 12.5F) || (gx > 15.5F && gx < 20.5F)) &&
                        ((gy > 2.5F && gy < 4.5F)  || (gy > 10.5F && gy < 12.5F));
                    // ASSEMBLY: 工程(number=1)消失在中央装配区方向
                    // 轨迹推断: 工程离开补给站朝中场出发 → 必然去装配区
                    // 红: x=3-16.5 且 vx>0.3 (朝中场/装配区方向)
                    // 蓝: x=11.5-25 且 vx<-0.3 (朝中场/装配区方向)
                    const bool is_engineer = (ghost.number == 1);
                    const bool in_assembly_approach =
                        ((gx > 3.0F && gx < 16.5F) && (gy > 3.0F && gy < 12.0F) && (vx > 0.3F)) ||
                        ((gx > 11.5F && gx < 25.0F) && (gy > 3.0F && gy < 12.0F) && (vx < -0.3F));
                    const bool heading_center = (vx > 0.3F) || (vx < -0.3F);
                    // 已在装配区中心 (速度衰减后仍保持 ASSEMBLY)
                    const bool in_assembly_zone =
                        (gx > 11.5F && gx < 16.5F && gy > 5.5F && gy < 9.5F);
                    // OUTPOST_AMBUSH: 英雄(number=0)消失在前哨站附近，速度较小
                    // 按 ghost 颜色门控: 蓝英雄只在 cross_tower 侧吊射区, 红英雄只在 self_tower 侧
                    const bool is_hero = (ghost.number == 0);
                    const float spd = std::sqrt(vx*vx + vy*vy);
                    const bool in_blue_outpost_zone = (ghost.color == 0) &&
                        (gx > hero_outpost_zone_lo_ && gx < hero_outpost_zone_hi_ &&
                         gy > hero_outpost_y_lo_   && gy < hero_outpost_y_hi_);
                    const bool in_red_outpost_zone  = (ghost.color == 2) &&
                        (gx > (arena_width_ - hero_outpost_zone_hi_) &&
                         gx < (arena_width_ - hero_outpost_zone_lo_) &&
                         gy > hero_outpost_y_lo_   && gy < hero_outpost_y_hi_);

                    if ((near_blue_base && heading_blue) || (near_red_base && heading_red)) {
                        ghost.behavior = RobotBehavior::CHARGING;
                    } else if (!is_engineer && in_tunnel_zone) {
                        ghost.behavior = RobotBehavior::TUNNEL;
                    } else if (is_engineer &&
                               (in_assembly_zone || (in_assembly_approach && heading_center))) {
                        // in_assembly_zone: 已在装配区中心(速度衰减后维持状态)
                        ghost.behavior = RobotBehavior::ASSEMBLY;
                    } else if (is_hero && spd < 1.5F &&
                               (in_blue_outpost_zone || in_red_outpost_zone)) {
                        ghost.behavior = RobotBehavior::OUTPOST_AMBUSH;
                    } else {
                        ghost.behavior = RobotBehavior::NORMAL;
                    }

                    // ── 运动能力持久观测 ────────────────────────────────────
                    // tunnel → CRAWLER; fly-ramp zone → LEGGED
                    // 飞坡区域: x≈13.5,y≈0.5 (红方飞坡) or x≈14.5,y≈14.5 (蓝方飞坡)
                    const bool in_flyramp = (gx > 12.5F && gx < 15.5F &&
                        ((gy < 2.0F) || (gy > 13.0F)));
                    // 梯形高地: 红方x=3-7,y=11-12.5; 蓝方x=21-25,y=2.5-4
                    const bool on_highland = (gx > 3.0F && gx < 7.5F && gy > 10.5F && gy < 13.0F) ||
                                             (gx > 20.5F && gx < 25.5F && gy > 2.0F && gy < 4.5F);

                    const uint8_t mob_key = static_cast<uint8_t>((ghost.color << 4) | ghost.number);
                    MobilityHint cur_mob = MobilityHint::UNKNOWN;
                    auto mit = robot_mobility_.find(mob_key);
                    if (mit != robot_mobility_.end()) cur_mob = mit->second;

                    MobilityHint new_mob = cur_mob;
                    if (in_tunnel_zone && cur_mob != MobilityHint::CRAWLER) {
                        new_mob = MobilityHint::CRAWLER;  // confirmed tunnel robot
                    } else if ((in_flyramp || on_highland) &&
                               cur_mob != MobilityHint::LEGGED) {
                        new_mob = MobilityHint::LEGGED;   // confirmed legged robot
                    }
                    if (new_mob != cur_mob) {
                        robot_mobility_[mob_key] = new_mob;
                        RCLCPP_INFO(get_logger(),
                            "MobilityHint %s%d → %s",
                            ghost.color == 2 ? "R" : "B", slot_to_robot_number(ghost.number),
                            new_mob == MobilityHint::CRAWLER ? "CRAWLER" :
                            new_mob == MobilityHint::LEGGED  ? "LEGGED"  : "WHEELED");
                    }
                    ghost.mobility = new_mob;
                }
                for (int s = 0; s < Kalman_filter_plus::N_IDS; ++s)
                    ghost.id_score[s] = KFs[i].id_score[s];
                auto& gv = ghost_tracks_;
                auto it = std::find_if(gv.begin(), gv.end(),
                    [&](const GhostTrack& g) {
                        return g.color == ghost.color && g.number == ghost.number;
                    });
                if (it != gv.end()) *it = ghost;
                else gv.push_back(ghost);
            }
            KFs.erase(KFs.begin() + i);
        } else {
            pcl::PointXYZRGB point;
            point.x = KFs[i].predict_point.x;
            point.y = KFs[i].predict_point.y;
            point.z = 1.5;
            int color = KFs[i].get_color();
            switch (color) {
            case 0:
                point.b = 255;
                break;

            case 2:
                point.r = 255;
                break;

            default:
                point.r = KFs[i].color[0];
                point.g = KFs[i].color[1];
                point.b = KFs[i].color[2];
                break;
            }
            cloud_filtered->points.push_back(point);
        }
    }
    cloud_filtered->header.frame_id = "rm_frame";
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud_filtered, output);
    output.header.frame_id = "rm_frame";
    output.header.stamp = msg->header.stamp;
    pub_->publish(output);
    auto  end_time = std::chrono::steady_clock::now();
    float dur_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                         end_time - now_time)
                         .count();
    if (pos_reinforce_enable_) {
        for (auto& kf : KFs) pos_reinforce(kf);
    }

    vision_interface::msg::DetectResult detect_msg;
    const int self_color_now = resolveSelfColor();
    // Spatial fallback slot counters for camera-uncalibrated operation
    int spatial_slot[3] = {0, 0, 0};  // [0]=blue slot, [2]=red slot
    for (const auto& kf : KFs) {
        // HKUST state machine: only publish CONFIRMED tracks
        // (TENTATIVE/INACTIVE = unverified noise; LOST = use ghost track instead)
        if (kf.get_state() != TrackState::CONFIRMED) continue;
        int color, number;
        if (kf.has_id()) {
            color = kf.get_color();
            if (color != 0 && color != 2) continue;  // tied/unknown
            number = kf.get_number();
        } else if (spatial_color_mode_ &&
                   (self_color_now == 0 || self_color_now == 2)) {
            // 相机不可用时按场地一侧推断颜色:
            //   红方区域 x < arena_width/2  → color=2 (red)
            //   蓝方区域 x > arena_width/2  → color=0 (blue)
            // (pos_reinforce 同样以 x<supply_x_depth 为红方补给区, 颜色侧定义一致)
            const float x = kf.predict_point.x;
            color = (x < arena_width_ * 0.5F) ? 2 : 0;
            number = spatial_slot[color]++;
            if (number >= kRobotSlotCount) continue;
        } else {
            continue;
        }
        if (number < 0 || number >= kRobotSlotCount) continue;
        if (color == 0) {
            detect_msg.blue_x[number] = kf.predict_point.x;
            detect_msg.blue_y[number] = kf.predict_point.y;
        } else {
            detect_msg.red_x[number] = kf.predict_point.x;
            detect_msg.red_y[number] = kf.predict_point.y;
        }
    }
    ghost_tracks_.erase(
        std::remove_if(ghost_tracks_.begin(), ghost_tracks_.end(),
            [&](const GhostTrack& g) {
                return (now_sec - g.timestamp) > static_cast<double>(ghost_ttl_s_);
            }),
        ghost_tracks_.end());

    // HP-based dead mask. id_score 槽: [0..4]=Blue, [5..9]=Red。
    // V1.3.0 0x0003 仅提供己方 HP, 故只能确认 "己方 (=self_color) 一侧" 哪些已死。
    // 敌方 track 拿不到 HP, 自然交给 track timeout 清理。
    const bool match_running = (match_info.match_time > 0);
    const int  self_c_dead   = resolveSelfColor();
    bool dead_slot[2 * kRobotSlotCount] = {};
    if (match_running && (self_c_dead == 0 || self_c_dead == 2)) {
        // self_c_dead == 2 (红) → 己方槽位 = [5..9]; self_c_dead == 0 (蓝) → [0..4]
        const int side_base = (self_c_dead == 2) ? kRobotSlotCount : 0;
        for (int slot = 0; slot < kRobotSlotCount; ++slot) {
            const int hp_idx = slot_to_hp_index(slot);
            if (hp_idx >= 0 && match_info.robot_hp[hp_idx] == 0)
                dead_slot[side_base + slot] = true;
        }
        // Zero out id_score for confirmed-dead robots so they can't be re-assigned
        for (auto& kf : KFs) {
            for (int s = 0; s < Kalman_filter_plus::N_IDS; ++s) {
                if (dead_slot[s]) kf.id_score[s] = 0.0F;
            }
        }
    }

    for (const auto& g : ghost_tracks_) {
        if (g.color != 0 && g.color != 2) continue;
        if (g.number < 0 || g.number >= kRobotSlotCount) continue;
        const int hp_slot = (g.color == 2) ? (kRobotSlotCount + g.number) : g.number;
        if (match_running && dead_slot[hp_slot]) continue;
        const float dt_ghost = static_cast<float>(now_sec - g.timestamp);
        if (dt_ghost > blind_timeout_s_) continue;  // beyond position-output window; kept for ID inheritance
        // Physics-correct decaying displacement: x(t) = x0 + v0/α*(1-exp(-α*t))
        // matches the velocity decay in update_predict_point (α=1.5)
        constexpr float alpha = 1.5F;
        const float disp_scale = (1.0F - std::exp(-alpha * dt_ghost)) / alpha;
        float gx = std::clamp(g.x + g.vx * disp_scale, 0.0F, arena_width_);
        float gy = std::clamp(g.y + g.vy * disp_scale, 0.0F, arena_height_);

        // 盲区猜点吸引 (HKUST/Ultra PointGuesser + 行为分类)
        // 速度衰减后，按 cos+距离评分向最优藏身点混合
        // CHARGING → 仅考虑基地充电区坐标; TUNNEL → 偏向隧道出口
        // 阴影区内: 立即开始snap (不等 snap_start_s), 因为已知激光不可见
        const bool in_shadow = shadow_enable_ && isBlindZone(gx, gy);
        const float eff_snap_start = in_shadow ? 0.0F : guess_snap_start_s_;
        if (guess_pts_enable_ && dt_ghost > eff_snap_start) {
            // ── 前哨被摧毁后, 敌方英雄必定进入吊射基地模式 ────────────────
            // V1.3.0 0x0003 仅提供己方 HP, robot_hp[6] 即己方前哨 (无论 self_color).
            // 我方前哨被毁 → 强制敌方英雄 ghost 为 OUTPOST_AMBUSH, 吸向吊射位。
            RobotBehavior eff_behavior = g.behavior;
            if (g.number == 0 && match_running) {
                const int self_c = resolveSelfColor();
                const bool ghost_is_enemy = (self_c >= 0 &&
                    g.color != static_cast<uint8_t>(self_c));
                if (ghost_is_enemy && match_info.robot_hp[6] == 0)
                    eff_behavior = RobotBehavior::OUTPOST_AMBUSH;
            }
            const std::string key = robotKey(g.color, g.number);
            auto it = guess_pts_.find(key);
            if (it != guess_pts_.end() && !it->second.empty()) {
                float best_score = -1e9F;
                float bx = gx, by = gy;
                for (const auto& p : it->second) {
                    // ── 行为过滤 ──────────────────────────────────────────
                    // CHARGING: 仅snap到基地边(x<5.5 或 x>22.5)的猜点
                    if (eff_behavior == RobotBehavior::CHARGING) {
                        const bool is_base_pt =
                            (p.first < supply_x_depth_ + 1.0F) ||
                            (p.first > arena_width_ - supply_x_depth_ - 1.0F);
                        if (!is_base_pt) continue;
                    }
                    // ASSEMBLY: 只snap到中央装配区猜点 (x=11-16, y=5-10)
                    // 能量机关正下方装配区, 工程装配/取矿核心活动区
                    if (eff_behavior == RobotBehavior::ASSEMBLY) {
                        const bool is_assembly_pt =
                            (p.first > 11.0F && p.first < 17.0F) &&
                            (p.second > 4.5F && p.second < 10.5F);
                        if (!is_assembly_pt) continue;
                    }
                    // OUTPOST_AMBUSH: 只考虑前哨站后方的吊射位 (按 ghost 颜色门控)
                    // 蓝英雄 → cross_tower阴影区 x=16-23; 红英雄 → self_tower阴影区 x=5-12
                    if (eff_behavior == RobotBehavior::OUTPOST_AMBUSH) {
                        const bool ok = (g.color == 0)
                            ? (p.first > 16.0F && p.first < 23.0F)
                            : (p.first >  5.0F && p.first < 12.0F);
                        if (!ok) continue;
                    }
                    // TUNNEL: 不过滤，仅通过余弦自然偏好出口方向
                    const float dx   = p.first  - gx;
                    const float dy   = p.second - gy;
                    const float dist = std::sqrt(dx * dx + dy * dy);
                    const float d_sc = std::exp(-dist * guess_d_factor_);
                    float cos_sim = 0.0F;
                    const float vlen = std::sqrt(g.vx * g.vx + g.vy * g.vy);
                    if (vlen > 0.05F && dist > 0.01F)
                        cos_sim = (g.vx * dx + g.vy * dy) / (vlen * dist);
                    // TUNNEL 模式加大方向权重 (强调出口方向)
                    const float cw = (g.behavior == RobotBehavior::TUNNEL)
                                     ? std::min(0.6F, guess_cos_factor_ * 3.0F)
                                     : guess_cos_factor_;
                    const float score = cw * cos_sim + (1.0F - cw) * d_sc;
                    if (score > best_score) { best_score = score; bx = p.first; by = p.second; }
                }
                if (best_score > -1e8F) {  // at least one candidate passed filter
                    // CHARGING/OUTPOST_AMBUSH: ×2 加快snap (目标位置固定)
                    // ASSEMBLY: ×1.5 中速snap (工程装配期间可能仍在移动)
                    float ramp =
                        (eff_behavior == RobotBehavior::CHARGING ||
                         eff_behavior == RobotBehavior::OUTPOST_AMBUSH)
                        ? guess_snap_ramp_ * 2.0F
                        : (eff_behavior == RobotBehavior::ASSEMBLY)
                        ? guess_snap_ramp_ * 1.5F
                        : guess_snap_ramp_;
                    // 阴影区内加速snap收敛到猜点
                    if (in_shadow) ramp *= shadow_snap_ramp_mult_;
                    const float snap = std::min(guess_snap_max_,
                        (dt_ghost - eff_snap_start) * ramp);
                    gx = gx * (1.0F - snap) + bx * snap;
                    gy = gy * (1.0F - snap) + by * snap;
                }
            }
        }

        if (g.color == 0 && detect_msg.blue_x[g.number] == 0.0F && detect_msg.blue_y[g.number] == 0.0F) {
            detect_msg.blue_x[g.number] = gx;
            detect_msg.blue_y[g.number] = gy;
        }
        if (g.color == 2 && detect_msg.red_x[g.number] == 0.0F && detect_msg.red_y[g.number] == 0.0F) {
            detect_msg.red_x[g.number] = gx;
            detect_msg.red_y[g.number] = gy;
        }
    }
    radar_detect_pub_->publish(detect_msg);

    vision_interface::msg::Radar2Sentry radar_msg;
    const int self_color = resolveSelfColor();
    if (self_color == 0) {
        for (int i = 0; i < kRobotSlotCount; i++) {
            radar_msg.radar_enemy_x[i] = detect_msg.red_x[i];
            radar_msg.radar_enemy_y[i] = detect_msg.red_y[i];
            radar_msg.radar_self_x[i] = detect_msg.blue_x[i];
            radar_msg.radar_self_y[i] = detect_msg.blue_y[i];
        }
    } else if (self_color == 2) {
        for (int i = 0; i < kRobotSlotCount; i++) {
            radar_msg.radar_enemy_x[i] = detect_msg.blue_x[i];
            radar_msg.radar_enemy_y[i] = detect_msg.blue_y[i];
            radar_msg.radar_self_x[i] = detect_msg.red_x[i];
            radar_msg.radar_self_y[i] = detect_msg.red_y[i];
        }
    } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "self_color unknown, skip /radar2sentry publish");
        return;
    }

    radar_pub_->publish(radar_msg);

    if (log_match_stats_) {
        // HKUST state machine distribution
        int n_inactive = 0, n_tentative = 0, n_confirmed = 0, n_lost = 0;
        for (const auto& kf : KFs) {
            switch (kf.get_state()) {
                case TrackState::INACTIVE:  n_inactive++;  break;
                case TrackState::TENTATIVE: n_tentative++; break;
                case TrackState::CONFIRMED: n_confirmed++; break;
                case TrackState::LOST:      n_lost++;      break;
            }
        }
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "tracks=%zu [INA=%d TEN=%d CONF=%d LOST=%d] lidar_in=%zu new=%zu updated=%zu ghosts=%zu",
            KFs.size(), n_inactive, n_tentative, n_confirmed, n_lost,
            cloud_xy->size(), lidar_new_tracks, lidar_updated_tracks,
            ghost_tracks_.size());
    }
}

void KalmanFilter::nms_tracks()
{
    if (nms_merge_r_ <= 0.0F || KFs.size() < 2) return;
    // Sort most-confirmed first so the keeper is always the more reliable track
    std::stable_sort(KFs.begin(), KFs.end(), [](const Kalman_filter_plus& a,
                                                 const Kalman_filter_plus& b) {
        return a.confirm_count_ > b.confirm_count_;
    });
    std::vector<bool> to_remove(KFs.size(), false);
    for (int i = 0; i < (int)KFs.size(); ++i) {
        if (to_remove[i]) continue;
        for (int j = i + 1; j < (int)KFs.size(); ++j) {
            if (to_remove[j]) continue;
            if (KFs[i].Distance(KFs[i].predict_point, KFs[j].predict_point) < nms_merge_r_) {
                // Absorb j's id_score and history into i
                for (int s = 0; s < Kalman_filter_plus::N_IDS; ++s)
                    KFs[i].id_score[s] = std::min(KFs[i].id_score_inc_ * 25.0F,
                        KFs[i].id_score[s] + KFs[j].id_score[s]);
                for (auto& h : KFs[j].detect_history)
                    KFs[i].detect_history.push_back(h);
                if ((int)KFs[i].detect_history.size() > KFs[i].max_history)
                    KFs[i].detect_history.erase(
                        KFs[i].detect_history.begin(),
                        KFs[i].detect_history.begin() +
                            (int)KFs[i].detect_history.size() - KFs[i].max_history);
                // 若 j 刚拿到新 LiDAR 而 i 没有, 用 force_position 把 i 的整套状态
                // (含 statePost 与 vx/vy) 推到 j 的位置, 否则下一次 predict 会把
                // i 旧的内部状态又算回去, 合并结果一帧内就消失。
                if (KFs[j].has_updated && !KFs[i].has_updated) {
                    KFs[i].force_position(KFs[j].predict_point.x, KFs[j].predict_point.y);
                    KFs[i].has_updated   = true;
                }
                KFs[i].confirm_count_ = std::min(5, KFs[i].confirm_count_ + KFs[j].confirm_count_ / 2);
                // HKUST state machine: keep best hit/miss counts when merging
                KFs[i].hit_count_  = std::max(KFs[i].hit_count_, KFs[j].hit_count_);
                KFs[i].miss_count_ = std::min(KFs[i].miss_count_, KFs[j].miss_count_);
                to_remove[j] = true;
            }
        }
    }
    for (int i = (int)KFs.size() - 1; i >= 0; --i)
        if (to_remove[i]) KFs.erase(KFs.begin() + i);
}

void KalmanFilter::prune_static_tracks()
{
    if (static_filter_dist_ <= 0.0F || static_filter_age_s_ <= 0.0F) return;
    for (int i = (int)KFs.size() - 1; i >= 0; --i) {
        if (KFs[i].confirm_count_ >= 2) continue;        // confirmed tracks are exempt
        if (KFs[i].track_age_s_ < static_filter_age_s_) continue; // too young
        // Compute max XY displacement from birth position across entire history
        float max_disp = 0.0F;
        if (KFs[i].history.size() >= 2) {
            const auto& birth = KFs[i].history.front().second;
            for (const auto& h : KFs[i].history) {
                const float dx = h.second.x - birth.x;
                const float dy = h.second.y - birth.y;
                const float d  = std::sqrt(dx * dx + dy * dy);
                if (d > max_disp) max_disp = d;
            }
        }
        if (max_disp < static_filter_dist_) {
            KFs.erase(KFs.begin() + i);  // silent removal: no ghost, no publish
        }
    }
}

void KalmanFilter::cap_track_count()
{
    if (max_tracks_ <= 0 || (int)KFs.size() <= max_tracks_) return;
    // Sort ascending by confirm_count so weakest tracks are at the front
    std::stable_sort(KFs.begin(), KFs.end(), [](const Kalman_filter_plus& a,
                                                 const Kalman_filter_plus& b) {
        return a.confirm_count_ < b.confirm_count_;
    });
    const int excess = (int)KFs.size() - max_tracks_;
    KFs.erase(KFs.begin(), KFs.begin() + excess);
}

void KalmanFilter::pos_reinforce(Kalman_filter_plus& kf) const
{
    if (!pos_reinforce_enable_) return;

    const float x = kf.predict_point.x;
    const float y = kf.predict_point.y;

    auto in_supply_y = [&](float py) -> bool {
        return (py >= supply_y_band_lo_ && py <= supply_y_band_hi_) ||
               (py >= (arena_height_ - supply_y_band_hi_) &&
                py <= (arena_height_ - supply_y_band_lo_));
    };

    const bool in_red_supply  = (x < supply_x_depth_)                       && in_supply_y(y);
    const bool in_blue_supply = (x > arena_width_ - supply_x_depth_)        && in_supply_y(y);

    if (in_red_supply || in_blue_supply) {
        const int eng_color = in_red_supply ? 2 : 0;
        for (int v = 0; v < engineer_reinforce_votes_; ++v)
            kf.detect_history.push_back({eng_color, 1});
        kf.update_id_score(eng_color, 1);
    }

    const float dx_red  = std::abs(x - outpost_x_);
    const float dx_blue = std::abs(x - (arena_width_ - outpost_x_));
    if (dx_red < outpost_zone_r_ || dx_blue < outpost_zone_r_) {
        const int hero_color = (dx_blue < dx_red) ? 0 : 2;
        for (int v = 0; v < hero_reinforce_votes_; ++v)
            kf.detect_history.push_back({hero_color, 0});
        kf.update_id_score(hero_color, 0);
    }

    if (kf.detect_history.size() > static_cast<size_t>(track_history_size_)) {
        kf.detect_history.erase(
            kf.detect_history.begin(),
            kf.detect_history.begin() +
                static_cast<int>(kf.detect_history.size()) - track_history_size_);
    }
}

rcl_interfaces::msg::SetParametersResult KalmanFilter::on_parameters_changed(
    const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "ok";

    float new_lidar_match_radius = lidar_match_radius_;
    float new_lidar_match_speed_gain = lidar_match_speed_gain_;
    float new_track_timeout_s = track_timeout_s_;
    int new_track_history_size = track_history_size_;
    float new_camera_time_threshold_s = camera_time_threshold_s_;
    float new_blind_timeout_s = blind_timeout_s_;
    bool new_log_match_stats = log_match_stats_;
    bool new_pos_reinforce_enable = pos_reinforce_enable_;
    float new_nms_merge_r = nms_merge_r_;
    float new_static_filter_dist = static_filter_dist_;
    float new_static_filter_age  = static_filter_age_s_;
    bool  new_spatial_color_mode = spatial_color_mode_;
    float new_id_score_inc    = id_score_inc_;
    float new_id_score_dec    = id_score_dec_;
    float new_id_score_decay  = id_score_decay_;
    float new_id_score_thresh = id_score_thresh_;
    float new_kf_cov_factor   = kf_cov_factor_;
    float new_kf_cov_freeze   = kf_cov_freeze_s_;
    float new_id_hi_mult      = id_score_hi_mult_;
    float new_cost_w_hist     = cost_w_hist_;
    float new_cost_w_botid    = cost_w_botid_;
    float new_cost_w_pos      = cost_w_pos_;
    float new_ghost_inherit_r = ghost_inherit_r_;
    float new_ghost_ttl       = ghost_ttl_s_;
    float new_guess_d_factor   = guess_d_factor_;
    float new_guess_cos_factor = guess_cos_factor_;
    float new_guess_snap_max   = guess_snap_max_;
    float new_guess_snap_ramp  = guess_snap_ramp_;
    float new_guess_snap_start = guess_snap_start_s_;

    for (const auto& p : parameters) {
        if (p.get_name() == "lidar_match_radius") {
            new_lidar_match_radius = static_cast<float>(p.as_double());
        } else if (p.get_name() == "lidar_match_speed_gain") {
            new_lidar_match_speed_gain = static_cast<float>(p.as_double());
        } else if (p.get_name() == "track_timeout_s") {
            new_track_timeout_s = static_cast<float>(p.as_double());
        } else if (p.get_name() == "track_history_size") {
            new_track_history_size = p.as_int();
        } else if (p.get_name() == "camera_time_threshold_s") {
            new_camera_time_threshold_s = static_cast<float>(p.as_double());
        } else if (p.get_name() == "blind_timeout_s") {
            new_blind_timeout_s = static_cast<float>(p.as_double());
        } else if (p.get_name() == "nms_merge_r") {
            new_nms_merge_r = static_cast<float>(p.as_double());
        } else if (p.get_name() == "static_filter_dist") {
            new_static_filter_dist = static_cast<float>(p.as_double());
        } else if (p.get_name() == "static_filter_age_s") {
            new_static_filter_age = static_cast<float>(p.as_double());
        } else if (p.get_name() == "spatial_color_mode") {
            new_spatial_color_mode = p.as_bool();
        } else if (p.get_name() == "log_match_stats") {
            new_log_match_stats = p.as_bool();
        } else if (p.get_name() == "pos_reinforce_enable") {
            new_pos_reinforce_enable = p.as_bool();
        } else if (p.get_name() == "id_score_inc") {
            new_id_score_inc = static_cast<float>(p.as_double());
        } else if (p.get_name() == "id_score_dec") {
            new_id_score_dec = static_cast<float>(p.as_double());
        } else if (p.get_name() == "id_score_decay") {
            new_id_score_decay = static_cast<float>(p.as_double());
        } else if (p.get_name() == "id_score_thresh") {
            new_id_score_thresh = static_cast<float>(p.as_double());
        } else if (p.get_name() == "kf_cov_factor") {
            new_kf_cov_factor = static_cast<float>(p.as_double());
        } else if (p.get_name() == "kf_cov_freeze_s") {
            new_kf_cov_freeze = static_cast<float>(p.as_double());
        } else if (p.get_name() == "id_score_hi_mult") {
            new_id_hi_mult = static_cast<float>(p.as_double());
        } else if (p.get_name() == "cost_w_hist") {
            new_cost_w_hist = static_cast<float>(p.as_double());
        } else if (p.get_name() == "cost_w_botid") {
            new_cost_w_botid = static_cast<float>(p.as_double());
        } else if (p.get_name() == "cost_w_pos") {
            new_cost_w_pos = static_cast<float>(p.as_double());
        } else if (p.get_name() == "ghost_inherit_r") {
            new_ghost_inherit_r = static_cast<float>(p.as_double());
        } else if (p.get_name() == "ghost_ttl_s") {
            new_ghost_ttl = static_cast<float>(p.as_double());
        } else if (p.get_name() == "guess_d_factor") {
            new_guess_d_factor = static_cast<float>(p.as_double());
        } else if (p.get_name() == "guess_cos_factor") {
            new_guess_cos_factor = static_cast<float>(p.as_double());
        } else if (p.get_name() == "guess_snap_max") {
            new_guess_snap_max = static_cast<float>(p.as_double());
        } else if (p.get_name() == "guess_snap_ramp") {
            new_guess_snap_ramp = static_cast<float>(p.as_double());
        } else if (p.get_name() == "guess_snap_start_s") {
            new_guess_snap_start = static_cast<float>(p.as_double());
        }
    }

    if (new_lidar_match_radius <= 0.0F || new_lidar_match_speed_gain <= 0.0F) {
        result.successful = false;
        result.reason = "lidar_match_radius and lidar_match_speed_gain must be > 0";
        return result;
    }
    if (new_track_timeout_s <= 0.0F || new_camera_time_threshold_s <= 0.0F) {
        result.successful = false;
        result.reason = "track_timeout_s and camera_time_threshold_s must be > 0";
        return result;
    }
    if (new_track_history_size < 5) {
        result.successful = false;
        result.reason = "track_history_size must be >= 5";
        return result;
    }

    lidar_match_radius_ = new_lidar_match_radius;
    lidar_match_speed_gain_ = new_lidar_match_speed_gain;
    track_timeout_s_ = new_track_timeout_s;
    track_history_size_ = new_track_history_size;
    camera_time_threshold_s_ = new_camera_time_threshold_s;
    blind_timeout_s_ = std::max(0.0F, new_blind_timeout_s);
    nms_merge_r_         = std::max(0.0F, new_nms_merge_r);
    static_filter_dist_  = std::max(0.0F, new_static_filter_dist);
    static_filter_age_s_  = std::max(0.0F, new_static_filter_age);
    spatial_color_mode_   = new_spatial_color_mode;
    log_match_stats_      = new_log_match_stats;
    pos_reinforce_enable_ = new_pos_reinforce_enable;
    id_score_inc_    = std::max(1.0F, new_id_score_inc);
    id_score_dec_    = std::max(0.0F, new_id_score_dec);
    id_score_decay_  = std::max(0.0F, new_id_score_decay);
    id_score_thresh_ = std::max(1.0F, new_id_score_thresh);
    kf_cov_factor_   = std::max(0.0F, new_kf_cov_factor);
    kf_cov_freeze_s_ = std::max(0.0F, new_kf_cov_freeze);
    id_score_hi_mult_ = std::clamp(new_id_hi_mult, 0.0F, 1.0F);
    cost_w_hist_      = std::max(0.0F, new_cost_w_hist);
    cost_w_botid_     = std::max(0.0F, new_cost_w_botid);
    cost_w_pos_       = std::max(0.0F, new_cost_w_pos);
    ghost_inherit_r_  = std::max(0.0F, new_ghost_inherit_r);
    ghost_ttl_s_      = std::max(1.0F, new_ghost_ttl);
    guess_d_factor_   = std::max(0.0F, new_guess_d_factor);
    guess_cos_factor_ = std::max(0.0F, new_guess_cos_factor);
    guess_snap_max_   = std::clamp(new_guess_snap_max, 0.0F, 1.0F);
    guess_snap_ramp_  = std::max(0.0F, new_guess_snap_ramp);
    guess_snap_start_s_ = std::max(0.0F, new_guess_snap_start);
    for (auto& kf : KFs) {
        kf.set_tuning(lidar_match_radius_, lidar_match_speed_gain_,
                      track_history_size_, camera_time_threshold_s_,
                      id_score_inc_, id_score_dec_, id_score_decay_,
                      id_score_thresh_, kf_cov_factor_, kf_cov_freeze_s_,
                      id_score_hi_mult_);
    }
    return result;
}

// ── 盲区猜点 YAML 加载 (HKUST/Ultra PointGuesser 移植) ─────────────────────
void KalmanFilter::load_guess_pts(const std::string& path)
{
    try {
        YAML::Node root = YAML::LoadFile(path);
        const auto& d_factor_node   = root["d_factor"];
        const auto& cos_factor_node = root["cos_factor"];
        const auto& snap_max_node   = root["snap_max"];
        const auto& snap_ramp_node  = root["snap_ramp"];
        const auto& snap_start_node = root["snap_start_s"];
        if (d_factor_node)   guess_d_factor_    = d_factor_node.as<float>();
        if (cos_factor_node) guess_cos_factor_  = cos_factor_node.as<float>();
        if (snap_max_node)   guess_snap_max_    = snap_max_node.as<float>();
        if (snap_ramp_node)  guess_snap_ramp_   = snap_ramp_node.as<float>();
        if (snap_start_node) guess_snap_start_s_= snap_start_node.as<float>();

        const auto& pts_node = root["guess_points"];
        if (!pts_node || !pts_node.IsMap()) {
            RCLCPP_WARN(this->get_logger(), "guess_pts: no 'guess_points' map in %s", path.c_str());
            return;
        }
        for (const auto& kv : pts_node) {
            const std::string robot_key = kv.first.as<std::string>();
            GuessPtList pts;
            for (const auto& pt : kv.second) {
                pts.emplace_back(pt[0].as<float>(), pt[1].as<float>());
            }
            guess_pts_[robot_key] = std::move(pts);
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "load_guess_pts failed (%s): %s", path.c_str(), e.what());
    }
}

// ── 阴影图加载 (config/outputs/shadow_red.bin + shadow_blue.bin) ──────────────
// 格式: raw uint8, 行优先, shape=(kShadowRows, kShadowCols), 1=可见 0=盲区
void KalmanFilter::loadShadowGrid(const std::string& dir)
{
    const std::string names[2] = { dir + "/shadow_red.bin", dir + "/shadow_blue.bin" };
    for (int i = 0; i < 2; ++i) {
        std::ifstream f(names[i], std::ios::binary);
        if (!f.is_open()) {
            RCLCPP_WARN(this->get_logger(), "shadow grid not found: %s", names[i].c_str());
            continue;
        }
        const size_t n = static_cast<size_t>(kShadowRows) * kShadowCols;
        shadow_grid_[i].resize(n);
        f.read(reinterpret_cast<char*>(shadow_grid_[i].data()), static_cast<std::streamsize>(n));
        if (!f) {
            RCLCPP_WARN(this->get_logger(), "shadow grid read error: %s", names[i].c_str());
            shadow_grid_[i].clear();
        } else {
            RCLCPP_INFO(this->get_logger(), "Shadow grid loaded: %s (%zu cells)", names[i].c_str(), n);
        }
    }
}

// ── 查询 (gx,gy) 是否在己方雷达盲区内 ─────────────────────────────────────────
// 返回 true = 盲区 (激光预期看不到), false = 可见区或shadow未加载
bool KalmanFilter::isBlindZone(float x, float y) const
{
    const int self_c = resolveSelfColor();
    // red team (2) → 用 shadow_red [0]; blue team (0) → 用 shadow_blue [1]
    const int idx = (self_c == 2) ? 0 : 1;
    if (shadow_grid_[idx].empty()) return false;
    const int ix = static_cast<int>(x / kShadowRes);
    const int iy = static_cast<int>(y / kShadowRes);
    if (ix < 0 || ix >= kShadowCols || iy < 0 || iy >= kShadowRows) return false;
    return shadow_grid_[idx][static_cast<size_t>(iy) * kShadowCols + ix] == 0;
}

}  // namespace tdt_radar
RCLCPP_COMPONENTS_REGISTER_NODE(tdt_radar::KalmanFilter)
