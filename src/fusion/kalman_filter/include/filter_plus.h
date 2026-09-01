#include <opencv2/core/hal/interface.h>
#include <algorithm>
#include <vector>
#include <opencv2/core/types.hpp>
#include <pcl/impl/point_types.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include "opencv2/opencv.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#pragma once

// ── 显式 4 状态机 (HKUST ENTERPRIZE 2025 论文 Algorithm 1 Step 5-6) ────────
// 派生自 confirm_count_ + last_time，仅用于日志/调试/外部观察，不改业务逻辑。
//   INACTIVE  : 初始或已失效；当前对应 confirm_count_=0 且 last_time>delete_time
//   TENTATIVE : 暂定；confirm_count_ ∈ [1, HIT_THRESHOLD)
//   CONFIRMED : 已确认；confirm_count_ ≥ HIT_THRESHOLD 且 last_time < delete_time
//   LOST      : 曾确认但当前丢失；confirm_count_ ≥ HIT_THRESHOLD 且 last_time ≥ delete_time
enum class TrackState : uint8_t {
    INACTIVE  = 0,
    TENTATIVE = 1,
    CONFIRMED = 2,
    LOST      = 3,
};

class Kalman_filter_plus {
private:
    cv::KalmanFilter KF;

public:
    // id_score[0..4]=Blue, [5..9]=Red
    static constexpr int N_IDS = 10;
    static constexpr int HIT_THRESHOLD = 2;  // 与现有 confirm_count_≥2 当作 CONFIRMED 一致

    float Distance(pcl::PointXY& a, pcl::PointXY& b)
    {
        return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
    }
    float get_time()
    {
        auto duration =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - timer);
        return duration.count() / 1000.0;
    }
    float                                        last_time = 0;
    float                                        camera_last_time = 0.0F;
    double                                       camera_time_threshold_ = 1.0;
    int                                          confirm_count_ = 0;
    int                                          hit_count_ = 0;      // 显式命中计数 (HKUST state machine)
    int                                          miss_count_ = 0;     // 显式丢失计数 (HKUST state machine)
    int                                          last_bot_id_ = -1;   // 上一次匹配的 detect short-term track ID
    float                                        track_age_s_ = 0.0F;
    std::chrono::steady_clock::time_point        timer;
    float                                        delete_time = 2.0;
    std::vector<std::pair<double, pcl::PointXY>> history;
    std::vector<std::pair<int, int>>             detect_history;
    int                                          max_history = 20;

    pcl::PointXY predict_point;
    float        detect_r = 1;
    float        car_speed = 2;
    float        car_max_speed = 2.5;
    cv::Scalar   color;
    bool         has_updated = false;
    cv::Mat      Q = cv::Mat::zeros(4, 4, CV_32F);
    cv::Mat      R = cv::Mat::zeros(2, 2, CV_32F);
    float        dt_ = 0.1f;
    float        sigma_q_x = 50.0f;
    float        sigma_q_y = 50.0f;
    float        sigma_r_x = 0.1f;
    float        sigma_r_y = 0.1f;

    float id_score[N_IDS] = {};
    float id_score_inc_    = 40.0F;
    float id_score_dec_    =  2.0F;
    float id_score_decay_  =  5.0F;
    float id_score_thresh_ = 60.0F;
    float id_score_hi_mult_= 0.1F;
    float kf_cov_factor_   = 0.002F;
    float kf_cov_freeze_s_ = 3.0F;

    Kalman_filter_plus(pcl::PointXY& input, rclcpp::Time time)
    {
        predict_point = input;
        history.push_back(std::make_pair(GetTimeByRosTime(time), input));
        timer = std::chrono::steady_clock::now();
        color = cv::Scalar(rand() % 255, rand() % 255, rand() % 255);
        int          stateSize = 4;
        int          measSize = 2;
        int          contrSize = 0;
        unsigned int type = CV_32F;
        KF.init(stateSize, measSize, contrSize, type);

        // 必须显式 zero-init: cv::Mat(rows,cols,type) 不保证清零, 否则 vx/vy
        // 是栈上残留内存, 新建 track 首次 predict 会飞向随机方向。
        cv::Mat state = cv::Mat::zeros(stateSize, 1, type);
        cv::Mat meas  = cv::Mat::zeros(measSize, 1, type);
        meas.at<float>(0) = input.x;
        meas.at<float>(1) = input.y;

        state.at<float>(0) = meas.at<float>(0);
        state.at<float>(2) = meas.at<float>(1);
        // state[1]=vx, state[3]=vy 已被 zeros() 置 0
        KF.statePost = state;
        KF.transitionMatrix = (cv::Mat_<float>(4, 4) << 1, dt_, 0, 0, 0, 1,
                               0, 0, 0, 0, 1, dt_, 0, 0, 0, 1);

        KF.measurementMatrix =
            (cv::Mat_<float>(2, 4) << 1, 0, 0, 0, 0, 0, 1, 0);

        KF.processNoiseCov =
            (cv::Mat_<float>(4, 4) << sigma_q_x * pow(dt_, 3) / 3,
             sigma_q_x * pow(dt_, 2) / 2, 0, 0, sigma_q_x * pow(dt_, 2) / 2,
             sigma_q_x * pow(dt_, 1), 0, 0, 0, 0,
             sigma_q_y * pow(dt_, 3) / 3, sigma_q_y * pow(dt_, 2) / 2, 0, 0,
             sigma_q_y * pow(dt_, 2) / 2, sigma_q_y * pow(dt_, 1));

        KF.measurementNoiseCov =
            (cv::Mat_<float>(2, 2) << sigma_r_x, 0, 0, sigma_r_y);

        setIdentity(KF.errorCovPost, cv::Scalar::all(1));
        has_updated = true;
    }

    ~Kalman_filter_plus() {}

    void set_tuning(float r, float speed_gain, int hist_size,
                    float cam_time_thresh,
                    float inc, float dec, float decay, float thresh,
                    float cov_factor, float cov_freeze,
                    float hi_mult)
    {
        detect_r             = r;
        car_max_speed        = speed_gain;
        max_history          = hist_size;
        camera_time_threshold_ = cam_time_thresh;
        id_score_inc_        = inc;
        id_score_dec_        = dec;
        id_score_decay_      = decay;
        id_score_thresh_     = thresh;
        kf_cov_factor_       = cov_factor;
        kf_cov_freeze_s_     = cov_freeze;
        id_score_hi_mult_    = hi_mult;
    }

    void update_id_score(int clr, int number)
    {
        if (number < 0 || number > 4) return;
        int slot = (clr == 2) ? (number + 5) : number;
        if (slot < 0 || slot >= N_IDS) return;
        id_score[slot] += id_score_inc_;
        for (int s = 0; s < N_IDS; ++s) {
            if (s != slot) id_score[s] -= id_score_dec_;
            if (id_score[s] < 0.0F) id_score[s] = 0.0F;
        }
    }

    // 返回 (color, number) 对应的归一化历史置信度 [0, 1+]
    // 用作 HKUST 论文中的 HistConf 软外观特征。
    // 注意：可能 >1 因 id_score 无硬上限；调用方应自己 clamp。
    float get_hist_conf(int clr, int number) const
    {
        if (number < 0 || number > 4) return 0.0F;
        int slot = (clr == 2) ? (number + 5) : number;
        if (slot < 0 || slot >= N_IDS) return 0.0F;
        if (id_score_thresh_ <= 1e-6F) return 0.0F;
        return id_score[slot] / id_score_thresh_;
    }

    // 派生 TrackState (基于显式 hit_count_ 和 miss_count_)
    // HKUST Algorithm 1 Step 5-6 风格状态机
    TrackState get_state() const
    {
        if (hit_count_ <= 0) {
            return TrackState::INACTIVE;
        }
        if (hit_count_ < HIT_THRESHOLD) {
            // TENTATIVE: 命中不足，若 miss_count 超过 hit_count 则回退到 INACTIVE
            return (miss_count_ >= hit_count_) ? TrackState::INACTIVE : TrackState::TENTATIVE;
        }
        if (miss_count_ < 10) {
            return TrackState::CONFIRMED;
        }
        return TrackState::LOST;
    }

    bool has_id() const
    {
        float mx = 0.0F;
        for (int s = 0; s < N_IDS; ++s)
            if (id_score[s] > mx) mx = id_score[s];
        return mx >= id_score_thresh_;
    }

    void get_id(int& out_color, int& out_number) const
    {
        float mx = 0.0F;
        int best = -1;
        for (int s = 0; s < N_IDS; ++s) {
            if (id_score[s] > mx) { mx = id_score[s]; best = s; }
        }
        if (best < 0 || mx < id_score_thresh_) {
            out_color = 1; out_number = -1; return;
        }
        if (best >= 5) { out_color = 2; out_number = best - 5; }
        else           { out_color = 0; out_number = best; }
    }

    float get_vx() const { return KF.statePost.at<float>(1); }
    float get_vy() const { return KF.statePost.at<float>(3); }

    void force_position(float x, float y)
    {
        // 用 history 中最近一个有效采样估计速度, 避免位置突变后残留旧 vx/vy
        // 导致下一次 predict 把陈旧速度叠加到新位置上。
        float vx = 0.0F, vy = 0.0F;
        if (!history.empty()) {
            const auto& last = history.back();
            const double now = last.first;
            // 找一个时间差在 (0, 1] 秒之内, 距离不为 0 的较早样本估速
            for (auto it = history.rbegin() + 1; it != history.rend(); ++it) {
                const double dt = now - it->first;
                if (dt <= 1e-3 || dt > 1.0) continue;
                vx = static_cast<float>((x - it->second.x) / dt);
                vy = static_cast<float>((y - it->second.y) / dt);
                // 限速保护, 防止 outlier
                const float vmax = std::max(1.0F, car_max_speed);
                vx = std::clamp(vx, -vmax, vmax);
                vy = std::clamp(vy, -vmax, vmax);
                break;
            }
        }
        KF.statePost.at<float>(0) = x;
        KF.statePost.at<float>(1) = vx;
        KF.statePost.at<float>(2) = y;
        KF.statePost.at<float>(3) = vy;
        predict_point.x = x;
        predict_point.y = y;
    }

    int get_color() const
    {
        // 0=blue, 2=red, 1=unknown (含: 无观测 / 红蓝平票)。
        // 之前 red==blue 时强制返 0 (blue), 会把噪声 track 一律误判为蓝队 hero,
        //  并污染 ghost color / 颜色一致性 gate / publish 颜色。
        if (detect_history.empty()) return 1;
        int red = 0, blue = 0;
        for (const auto& e : detect_history) {
            if (e.first == 0)      ++blue;
            else if (e.first == 2) ++red;
            // 其他 (理论上不应入队的 1) 直接跳过
        }
        if (blue == 0 && red == 0) return 1;
        if (red > blue) return 2;
        if (blue > red) return 0;
        return 1;  // tie → unknown
    }

    int get_number() const
    {
        const int color = get_color();
        // unknown / tie → 没有可信编号, 返回 -1 让上层丢弃 (避免默认 0=hero 假阳性)。
        if (color != 0 && color != 2) return -1;
        std::map<int, int> number_map;
        for (const auto& e : detect_history) {
            if (e.first == color) ++number_map[e.second];
        }
        int max_count = 0;
        int max_number = -1;
        for (const auto& entry : number_map) {
            if (entry.second > max_count) {
                max_count  = entry.second;
                max_number = entry.first;
            }
        }
        return max_number;
    }

    void update(pcl::PointXY& input, rclcpp::Time time)
    {
        cv::Mat meas = cv::Mat::zeros(2, 1, CV_32F);
        meas.at<float>(0) = input.x;
        meas.at<float>(1) = input.y;
        KF.correct(meas);
        predict_point.x = KF.statePost.at<float>(0);
        predict_point.y = KF.statePost.at<float>(2);
        has_updated = true;
        last_time = 0;
        auto temp_point = input;
        history.push_back(
            std::make_pair(GetTimeByRosTime(time), temp_point));
        if (history.size() > max_history) {
            history.erase(history.begin());
        }
    }

    void update_camera(pcl::PointXY& input, rclcpp::Time time)
    {
        history.push_back(std::make_pair(GetTimeByRosTime(time), input));
        if (history.size() > max_history) {
            history.erase(history.begin());
        }
        force_position(input.x, input.y);
        has_updated = true;
        camera_last_time = 0.0F;
        last_time = 0.0F;
    }

    void update_predict_point()
    {
        dt_ = get_time();
        timer = std::chrono::steady_clock::now();
        do_predict();
    }

    void update_predict_point(double /*ros_time_sec*/)
    {
        dt_ = get_time();
        timer = std::chrono::steady_clock::now();
        do_predict();
    }

    float match_and_cost(pcl::PointXY& input)
    {
        float d = Distance(predict_point, input);
        if (d < car_max_speed * dt_ + detect_r) return d;
        return -1.0F;
    }

    bool match(pcl::PointXY& input)
    {
        if (Distance(predict_point, input) <
            car_max_speed * dt_ + detect_r) {
            return true;
        } else {
            return false;
        }
    }

    void camera_match(rclcpp::Time& time, pcl::PointXY& input, int color,
                      int number)
    {
        const double TIME_THRESHOLD = 1.0f;
        double       input_time = GetTimeByRosTime(time);
        double       differ_time = 1000;
        pcl::PointXY match_point;
        for (auto& point : history) {
            auto differ = abs(point.first - input_time);
            if (differ < differ_time) {
                differ_time = differ;
                match_point = point.second;
            }
        }
        if (differ_time > TIME_THRESHOLD) {
            return;
        }
        if (Distance(match_point, input) < detect_r) {
            detect_history.push_back(std::make_pair(color, number));
            if (detect_history.size() > max_history) {
                detect_history.erase(detect_history.begin());
            }
        }
    }
    static double GetTimeByRosTime(rclcpp::Time& ros_time)
    {
        double ros_time_value = ros_time.nanoseconds() / 1e9;
        return ros_time_value;
    }

private:
    void do_predict()
    {
        // 用本帧实际 dt 重写转移矩阵与过程噪声 (构造时 dt=0.1 只是占位)
        KF.transitionMatrix.at<float>(0, 1) = dt_;
        KF.transitionMatrix.at<float>(2, 3) = dt_;
        const float dt2 = dt_ * dt_;
        const float dt3 = dt2 * dt_;
        KF.processNoiseCov.at<float>(0, 0) = sigma_q_x * dt3 / 3.0f;
        KF.processNoiseCov.at<float>(0, 1) = sigma_q_x * dt2 / 2.0f;
        KF.processNoiseCov.at<float>(1, 0) = sigma_q_x * dt2 / 2.0f;
        KF.processNoiseCov.at<float>(1, 1) = sigma_q_x * dt_;
        KF.processNoiseCov.at<float>(2, 2) = sigma_q_y * dt3 / 3.0f;
        KF.processNoiseCov.at<float>(2, 3) = sigma_q_y * dt2 / 2.0f;
        KF.processNoiseCov.at<float>(3, 2) = sigma_q_y * dt2 / 2.0f;
        KF.processNoiseCov.at<float>(3, 3) = sigma_q_y * dt_;
        auto result = KF.predict();
        last_time += dt_;
        camera_last_time += dt_;
        track_age_s_ += dt_;
        predict_point.x = result.at<float>(0);
        predict_point.y = result.at<float>(2);
        for (int s = 0; s < N_IDS; ++s) {
            float mult = (id_score[s] >= id_score_thresh_) ? id_score_hi_mult_ : 1.0F;
            id_score[s] -= id_score_decay_ * dt_ * mult;
            if (id_score[s] < 0.0F) id_score[s] = 0.0F;
        }
    }
};
