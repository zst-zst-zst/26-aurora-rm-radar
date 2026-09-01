#include <fstream>
#include <std_msgs/msg/u_int8.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_interface/msg/detect_result.hpp>
#include <vision_interface/msg/match_info.hpp>
#include <vision_interface/msg/radar_warn.hpp>
#include <vision_interface/robot_slots.h>
namespace tdt_radar {
namespace {
static constexpr int   kMapPixelsPerMeter = 25;
static constexpr int   kSidebarWidthPx    = 210;

int slotToDisplayId(int slot_index)
{
    if (slot_index == kSentrySlot) return 7;
    return slot_index + 1;
}

std::string slotToDisplayText(int slot_index)
{
    return std::to_string(slotToDisplayId(slot_index));
}
}  // namespace

class DebugMap : public rclcpp::Node {
public:
    explicit DebugMap(const rclcpp::NodeOptions& options)
        : Node("debug_map", options)
    {
        map_image_path_ = this->declare_parameter<std::string>("map_image", "config/map/map.jpg");
        arena_width_ = this->declare_parameter<float>("map_width", 28.0F);
        arena_height_ = this->declare_parameter<float>("map_height", 15.0F);
        input_is_self_frame_ =
            this->declare_parameter<bool>("input_is_self_frame", false);
        self_color_override_ =
            this->declare_parameter<int>("self_color_override", -1);
        // 双倍易伤: 永远全自动 — 一旦 referee 给出可用机会且场上有有效标记,
        // 立即下发触发命令; 不再保留 smart/hint 策略和威胁线。

        for (int i = 0; i < kRobotSlotCount; i++) {
            blue_time[i] = 0.0;
            red_time[i] = 0.0;
            blue_kalman_time[i] = -9999.0;
            red_kalman_time[i] = -9999.0;
            blue_camera_time[i] = -9999.0;
            red_camera_time[i]  = -9999.0;
            blue_update[i] = 0.0;
            red_update[i] = 0.0;
            blue_point[i] = cv::Point2f(0, 0);
            red_point[i] = cv::Point2f(0, 0);
            blue_vel[i]   = cv::Point2f(0, 0);
            red_vel[i]    = cv::Point2f(0, 0);
            blue_behavior[i] = 0;
            red_behavior[i]  = 0;
        }

        detect_result_sub =
            this->create_subscription<vision_interface::msg::DetectResult>(
                "/kalman_detect", 10,
                std::bind(&DebugMap::callback, this,
                          std::placeholders::_1));
        camera_detect_sub =
            this->create_subscription<vision_interface::msg::DetectResult>(
                "/resolve_result", rclcpp::SensorDataQoS(),
                std::bind(&DebugMap::camera_callback, this,
                          std::placeholders::_1));
        map = cv::imread(map_image_path_);
        if (map.empty()) {
            RCLCPP_WARN(this->get_logger(),
                        "Failed to load map image: %s, using blank canvas",
                        map_image_path_.c_str());
            map = cv::Mat::zeros(750, 1400, CV_8UC3);
        }
        match_info_sub =
            this->create_subscription<vision_interface::msg::MatchInfo>(
                "/match_info", 10,
                std::bind(&DebugMap::save_match_info, this,
                          std::placeholders::_1));
        lidar_warn_sub =
            this->create_subscription<vision_interface::msg::RadarWarn>(
                "/lidar_detect", 10,
                [this](const vision_interface::msg::RadarWarn::SharedPtr msg) {
                    lidar_warn_ = *msg;
                });
        debug_map_pub =
            this->create_publisher<sensor_msgs::msg::Image>("/map_2d", 10);
        debug_points_pub =
            this->create_publisher<vision_interface::msg::DetectResult>("/debug_map_points", 10);
        decision_pub_ =
            this->create_publisher<std_msgs::msg::UInt8>(
                "/radar/decision_request", 10);
        { cv::Mat tmp; cv::resize(map, tmp,
            cv::Size(static_cast<int>(arena_width_ * kMapPixelsPerMeter),
                     static_cast<int>(arena_height_ * kMapPixelsPerMeter))); map = std::move(tmp); }
        // Keep the debug window alive even when no detection topics are publishing yet.
        ui_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DebugMap::show_map, this));
    }

    int resolveSelfColor() const
    {
        if (self_color_override_ == 0 || self_color_override_ == 2) {
            return self_color_override_;
        }
        if (match_info.self_color == 0 || match_info.self_color == 2) {
            return match_info.self_color;
        }
        return -1;
    }

    cv::Point2f normalizeToGlobal(const cv::Point2f& p) const
    {
        if (!input_is_self_frame_) {
            return p;
        }

        const int self_color = resolveSelfColor();
        if (self_color == 0) {
            return cv::Point2f(arena_width_ - p.x, arena_height_ - p.y);
        }
        return p;
    }

    void save_match_info(
        const std::shared_ptr<vision_interface::msg::MatchInfo> msg)
    {
        // 协议合法值仅 0=蓝 / 2=红, 其它视为未知, 交给 resolveSelfColor() 处理
        this->match_info = *msg;
    }

    void show_map()
    {
        if (map.empty()) return;
        auto   now_time = std::chrono::system_clock::now();
        double time = std::chrono::duration_cast<std::chrono::milliseconds>(
                          now_time.time_since_epoch()).count() / 1000.0;

        static constexpr float kPxPerM = static_cast<float>(kMapPixelsPerMeter);
        static constexpr int   kSidebarW = kSidebarWidthPx;
        const int map_w = map.cols;
        const int map_h = map.rows;

        // Full canvas: map on left, sidebar on right
        cv::Mat canvas(map_h, map_w + kSidebarW, CV_8UC3, cv::Scalar(20, 20, 30));
        cv::Mat map_roi = canvas(cv::Rect(0, 0, map_w, map_h));
        map.copyTo(map_roi);

        // World → pixel
        auto w2p = [&](float x, float y) -> cv::Point {
            return cv::Point(static_cast<int>(x * kPxPerM),
                             static_cast<int>((arena_height_ - y) * kPxPerM));
        };

        // ── 1. Blind zone overlays (semi-transparent) ──────────────────────
        {
            cv::Mat ov = map_roi.clone();
            auto zone = [&](float x1, float y1, float x2, float y2, cv::Scalar c) {
                cv::rectangle(ov, w2p(x1, y2), w2p(x2, y1), c, -1);
            };
            // Outpost shadows
            zone( 9.2F,  5.0F, 16.0F, 10.0F, cv::Scalar(  0,  40, 100));  // red outpost
            zone(12.0F,  5.0F, 18.8F, 10.0F, cv::Scalar(100,  40,   0));  // blue outpost
            // Tunnel zones
            zone(10.0F,  2.5F, 15.5F,  5.5F, cv::Scalar( 80,   0,  80));
            zone(10.0F,  9.5F, 15.5F, 12.5F, cv::Scalar( 80,   0,  80));
            // Trapezoid highlands
            zone( 3.0F, 11.0F,  7.0F, 12.5F, cv::Scalar(  0,  70,  70));
            zone(21.0F,  2.5F, 25.0F,  4.0F, cv::Scalar(  0,  70,  70));
            // self_tower (10.82,3.66,1.87m): 热力图实测冷点 x=10-16, y=2.5-4.5
            // 阴影从塔本身向远处延伸, 覆盖塔后方到前哨战区域
            zone(10.0F,  2.5F, 16.5F,  4.5F, cv::Scalar(  0,  55, 110));
            // cross_tower (17.18,11.34,1.87m): 热力图实测冷点 x=17-21, y=10-12.5
            // 注意: x>21区域热力图并不冷, 不延伸太远
            zone(16.0F, 10.0F, 21.5F, 12.5F, cv::Scalar(130,  65,   0));
            cv::addWeighted(ov, 0.30, map_roi, 0.70, 0, map_roi);
        }

        // ── 2. Landmark outlines ───────────────────────────────────────────
        // Outpost rings
        cv::circle(map_roi, w2p( 9.2F, 7.5F), 14, cv::Scalar( 60, 120, 255), 2);
        cv::circle(map_roi, w2p(18.8F, 7.5F), 14, cv::Scalar(255, 120,  60), 2);
        // Towers
        cv::rectangle(map_roi, w2p(10.5F, 3.3F), w2p(11.2F, 4.1F), cv::Scalar(  0, 220, 255), 1);
        cv::rectangle(map_roi, w2p(16.8F,11.0F), w2p(17.5F,11.7F), cv::Scalar(255, 220,   0), 1);
        // Center tech core (hollow: just outline)
        cv::rectangle(map_roi, w2p(13.1F, 6.6F), w2p(14.9F, 8.4F), cv::Scalar(200, 200, 200), 1);
        // Midline
        cv::line(map_roi, w2p(14.0F, 0.0F), w2p(14.0F, 15.0F), cv::Scalar(60, 60, 60), 1);

        // ── 3. Draw robots ─────────────────────────────────────────────────
        static const int   kRadii[kRobotSlotCount] = {8, 6, 5, 5, 7};  // hero/sentry larger

        for (int i = 0; i < kRobotSlotCount; i++) {
            // ── Blue robots (color=0, drawn RED on field since enemy of red team)
            // In global frame: blue robots are on the blue side (x>14)
            if (blue_point[i].x > 0.1F || blue_point[i].y > 0.1F) {
                const double age_k = time - blue_time[i];
                const double age_c = time - blue_camera_time[i];
                const bool live  = (age_k < 1.5 && age_c < 2.0);
                const bool ghost = (!live && age_k < 30.0 && blue_time[i] > 0.0);
                if (live || ghost) {
                    const cv::Point pt = w2p(blue_point[i].x, blue_point[i].y);
                    const int r = kRadii[i];
                    if (live) {
                        cv::circle(map_roi, pt, r, cv::Scalar(180,  40,  40), -1);
                        cv::circle(map_roi, pt, r, cv::Scalar(255, 180, 180),  1);
                        cv::putText(map_roi, slotToDisplayText(i),
                                    cv::Point(pt.x - 4, pt.y + 4),
                                    cv::FONT_HERSHEY_SIMPLEX, 0.38,
                                    cv::Scalar(255, 255, 255), 1);
                        // Velocity arrow
                        const float vlen = std::sqrt(blue_vel[i].x*blue_vel[i].x +
                                                     blue_vel[i].y*blue_vel[i].y);
                        if (vlen > 0.4F) {
                            cv::arrowedLine(map_roi, pt,
                                cv::Point(pt.x + static_cast<int>(blue_vel[i].x*kPxPerM*0.35F),
                                          pt.y - static_cast<int>(blue_vel[i].y*kPxPerM*0.35F)),
                                cv::Scalar(80, 200, 255), 2, 8, 0, 0.35);
                        }
                    } else {
                        const float fade = std::max(0.15F, 1.0F - static_cast<float>(age_k - 1.5) / 28.5F);
                        const cv::Scalar gc(static_cast<uchar>(100*fade), static_cast<uchar>(20*fade),
                                            static_cast<uchar>(20*fade));
                        for (int a = 0; a < 360; a += 45)
                            cv::ellipse(map_roi, pt, cv::Size(r, r), 0, a, a+22, gc, 1);
                        cv::putText(map_roi, slotToDisplayText(i),
                                    cv::Point(pt.x - 4, pt.y + 4),
                                    cv::FONT_HERSHEY_SIMPLEX, 0.34, gc, 1);
                    }
                }
            }
            // ── Red robots (color=2, drawn BLUE on field)
            if (red_point[i].x > 0.1F || red_point[i].y > 0.1F) {
                const double age_k = time - red_time[i];
                const double age_c = time - red_camera_time[i];
                const bool live  = (age_k < 1.5 && age_c < 2.0);
                const bool ghost = (!live && age_k < 30.0 && red_time[i] > 0.0);
                if (live || ghost) {
                    const cv::Point pt = w2p(red_point[i].x, red_point[i].y);
                    const int r = kRadii[i];
                    if (live) {
                        cv::circle(map_roi, pt, r, cv::Scalar( 40,  40, 180), -1);
                        cv::circle(map_roi, pt, r, cv::Scalar(180, 180, 255),  1);
                        cv::putText(map_roi, slotToDisplayText(i),
                                    cv::Point(pt.x - 4, pt.y + 4),
                                    cv::FONT_HERSHEY_SIMPLEX, 0.38,
                                    cv::Scalar(255, 255, 255), 1);
                        const float vlen = std::sqrt(red_vel[i].x*red_vel[i].x +
                                                     red_vel[i].y*red_vel[i].y);
                        if (vlen > 0.4F) {
                            cv::arrowedLine(map_roi, pt,
                                cv::Point(pt.x + static_cast<int>(red_vel[i].x*kPxPerM*0.35F),
                                          pt.y - static_cast<int>(red_vel[i].y*kPxPerM*0.35F)),
                                cv::Scalar(255, 200, 80), 2, 8, 0, 0.35);
                        }
                    } else {
                        const float fade = std::max(0.15F, 1.0F - static_cast<float>(age_k - 1.5) / 28.5F);
                        const cv::Scalar gc(static_cast<uchar>(20*fade), static_cast<uchar>(20*fade),
                                            static_cast<uchar>(100*fade));
                        for (int a = 0; a < 360; a += 45)
                            cv::ellipse(map_roi, pt, cv::Size(r, r), 0, a, a+22, gc, 1);
                        cv::putText(map_roi, slotToDisplayText(i),
                                    cv::Point(pt.x - 4, pt.y + 4),
                                    cv::FONT_HERSHEY_SIMPLEX, 0.34, gc, 1);
                    }
                }
            }
        }

        // ── 4. Warnings on map ─────────────────────────────────────────────
        if (lidar_warn_.dart_state == 1)
            cv::putText(map_roi, "!! DART !!", cv::Point(10, 28),
                        cv::FONT_HERSHEY_DUPLEX, 0.75, cv::Scalar(0, 0, 255), 2);
        if (lidar_warn_.fly_state >= 1) {
            static constexpr const char* kFly[] = {"","UAV:UP","UAV:MID","UAV:ALARM"};
            static const cv::Scalar kFlyC[] = {{},{0,220,255},{0,140,255},{0,0,255}};
            const int fs = std::min(3, static_cast<int>(lidar_warn_.fly_state));
            cv::putText(map_roi, kFly[fs], cv::Point(10, 58),
                        cv::FONT_HERSHEY_DUPLEX, 0.75, kFlyC[fs], 2);
        }
        // Scale bar
        cv::line(map_roi, cv::Point(6, map_h-12), cv::Point(6+static_cast<int>(5*kPxPerM), map_h-12),
                 cv::Scalar(180,180,180), 1);
        cv::putText(map_roi, "5m", cv::Point(8, map_h-3), cv::FONT_HERSHEY_SIMPLEX,
                    0.38, cv::Scalar(180,180,180), 1);

        // ── 5. Sidebar ─────────────────────────────────────────────────────
        cv::Mat sidebar = canvas(cv::Rect(map_w, 0, kSidebarW, map_h));
        int sy = 18;
        auto sput = [&](const std::string& s, cv::Scalar c = cv::Scalar(200,200,200),
                        float sc = 0.36F, int th = 1) {
            cv::putText(sidebar, s, cv::Point(6, sy), cv::FONT_HERSHEY_SIMPLEX, sc, c, th);
            sy += static_cast<int>(sc * 22 + 4);
        };
        auto sdiv = [&]() {
            cv::line(sidebar, cv::Point(3, sy), cv::Point(kSidebarW-3, sy),
                     cv::Scalar(55, 55, 75), 1);
            sy += 7;
        };

        const int self_color = resolveSelfColor();
        sput(self_color == 2 ? "TEAM: RED" :
             self_color == 0 ? "TEAM: BLUE" : "TEAM: ?",
             self_color == 2 ? cv::Scalar(60,60,255) :
             self_color == 0 ? cv::Scalar(255,110,60) : cv::Scalar(150,150,150),
             0.44F, 2);

        // Match time
        const int mt = match_info.match_time;
        char tbuf[32];
        if      (mt < -100) snprintf(tbuf, sizeof(tbuf), "OFFLINE");
        else if (mt <    0) snprintf(tbuf, sizeof(tbuf), "T-%ds", -mt);
        else                snprintf(tbuf, sizeof(tbuf), "%d:%02d", mt/60, mt%60);
        sput(std::string("TIME: ") + tbuf, cv::Scalar(220,220,80), 0.40F);

        // Double vuln
        const uint8_t opp    = match_info.ultimate & 0x03U;
        const bool vuln_on   = (match_info.ultimate >> 2U) & 0x01U;
        sput((vuln_on ? "VULN ACTIVE x" : "VULN avail x") + std::to_string(opp),
             vuln_on ? cv::Scalar(0,60,255) : cv::Scalar(180,180,50), 0.34F);
        sdiv();

        // Enemy robots (无 HP: V1.3.0 0x0003 只提供己方 HP, 敌方 HP 协议层不可见)
        const auto& en_tm  = (self_color == 0) ? red_time     : blue_time;
        const auto& en_beh = (self_color == 0) ? red_behavior : blue_behavior;
        sput("ENEMY:", cv::Scalar(140,200,255), 0.34F);
        for (int i = 0; i < kRobotSlotCount; i++) {
            const uint8_t mark = match_info.marks[i];
            const double  age  = time - en_tm[i];
            static const char* kBeh[] = {""," C"," T"," O"};  // NORMAL/CHARGING/TUNNEL/OUTPOST
            char row[48];
            snprintf(row, sizeof(row), "%s %s%s",
                slotToDisplayText(i).c_str(),
                mark ? "[M]" : "   ",
                kBeh[std::min(3u, static_cast<unsigned>(en_beh[i]))]);
            sput(row,
                mark >= 1 ? cv::Scalar(0,80,255)    :
                age < 1.5 ? cv::Scalar(60,255,80)   :
                age < 30  ? cv::Scalar(200,200,80)  : cv::Scalar(100,100,100),
                0.30F);
        }
        sdiv();

        // Ally robots — V1.3.0 0x0003 已方 HP 一律放在 robot_hp[0..7]
        const auto& al_tm = (self_color == 0) ? blue_time  : red_time;
        sput("ALLY:", cv::Scalar(80,255,180), 0.34F);
        for (int i = 0; i < kRobotSlotCount; i++) {
            const int hp_idx  = slot_to_hp_index(i);
            const uint16_t hp  = (hp_idx >= 0 && hp_idx < 16) ? match_info.robot_hp[hp_idx] : 0;
            const double  age = time - al_tm[i];
            char row[32];
            snprintf(row, sizeof(row), "%s HP:%-3d",
                     slotToDisplayText(i).c_str(), hp);
            sput(row,
                hp == 0   ? cv::Scalar(50,50,50)   :
                age < 2.0 ? cv::Scalar(60,200,80)  : cv::Scalar(100,100,100),
                0.30F);
        }
        // 前哨 / 基地 HP (也来自 robot_hp[6]/[7])
        char outp_row[32], base_row[32];
        const uint16_t op_hp   = match_info.robot_hp[6];
        const uint16_t base_hp = match_info.robot_hp[7];
        snprintf(outp_row, sizeof(outp_row), "OP HP:%-3d", op_hp);
        snprintf(base_row, sizeof(base_row), "BS HP:%-3d", base_hp);
        sput(outp_row,
             op_hp == 0 ? cv::Scalar(60,60,60)
                        : op_hp < 200 ? cv::Scalar(60,60,255)
                                      : cv::Scalar(60,200,80),
             0.30F);
        sput(base_row,
             base_hp == 0 ? cv::Scalar(60,60,60)
                          : base_hp < 200 ? cv::Scalar(60,60,255)
                                          : cv::Scalar(60,200,80),
             0.30F);
        sdiv();

        sput("STRAT: AUTO (always immediate)", cv::Scalar(160,160,160), 0.32F);
        sput("r/b color  v vuln  h/H heat  q quit", cv::Scalar(80,80,80), 0.28F);

        // ── 6. Heatmap overlay (press h to toggle) ─────────────────────────
        if (show_heatmap_) {
            // 重新渲染热力图 (每2秒最多一次)
            if (time - heatmap_dirty_t_ > 2.0 || heatmap_overlay_.empty()) {
                heatmap_dirty_t_ = time;
                heatmap_overlay_ = cv::Mat::zeros(map_h, map_w, CV_8UC3);
                // 找最大值用于归一化
                float max_e = 1.0F, max_a = 1.0F;
                for (int iy = 0; iy < kHY; ++iy)
                    for (int ix = 0; ix < kHX; ++ix) {
                        max_e = std::max(max_e, enemy_heat_[iy][ix]);
                        max_a = std::max(max_a, ally_heat_[iy][ix]);
                    }
                // 用 float 边界, 避免 int(0.5*25)=12 截断导致右侧/上下少几个像素。
                const float cell_pxf = kHR * kPxPerM;
                for (int iy = 0; iy < kHY; ++iy) {
                    for (int ix = 0; ix < kHX; ++ix) {
                        const float fe = enemy_heat_[iy][ix] / max_e;
                        const float fa = ally_heat_[iy][ix]  / max_a;
                        if (fe < 0.01F && fa < 0.01F) continue;
                        // 像素坐标: iy=0是y=0 (底部), 显示时翻转y
                        const int px  = static_cast<int>(ix * cell_pxf);
                        const int px2 = static_cast<int>((ix + 1) * cell_pxf);
                        const int py2 = map_h - static_cast<int>(iy * cell_pxf);
                        const int py  = map_h - static_cast<int>((iy + 1) * cell_pxf);
                        if (px < 0 || py < 0 || px2 > map_w || py2 > map_h) continue;
                        const cv::Rect cell(px, py, px2 - px, py2 - py);
                        // 敌方=红热图, 己方=蓝热图
                        // 同时有值的地方叠加为紫色/白色
                        const uchar re = static_cast<uchar>(std::min(255.0F, fe * 220 + 35));
                        const uchar ba = static_cast<uchar>(std::min(255.0F, fa * 220 + 35));
                        cv::Mat roi = heatmap_overlay_(cell);
                        roi.setTo(cv::Scalar(ba/2, 0, re/2));
                    }
                }
            }
            // 叠加热力图
            cv::addWeighted(heatmap_overlay_, 0.55, map_roi, 0.45, 0, map_roi);
            // 图例
            cv::putText(map_roi, "HEATMAP  RED=enemy  BLUE=ally",
                        cv::Point(8, map_h - 28), cv::FONT_HERSHEY_SIMPLEX,
                        0.45, cv::Scalar(200, 200, 200), 1);
            cv::putText(map_roi, "H=clear  h=hide",
                        cv::Point(8, map_h - 14), cv::FONT_HERSHEY_SIMPLEX,
                        0.40, cv::Scalar(140, 140, 140), 1);
        }

        // ── 7. Keyboard ────────────────────────────────────────────────────
        const int key = cv::waitKey(1) & 0xFF;
        if      (key == 'q' || key == 27) { rclcpp::shutdown(); }
        else if (key == 'r') { self_color_override_ = 2;
                               RCLCPP_INFO(get_logger(), "Color→RED"); }
        else if (key == 'b') { self_color_override_ = 0;
                               RCLCPP_INFO(get_logger(), "Color→BLUE"); }
        else if (key == 'c') { self_color_override_ = -1;
                               RCLCPP_INFO(get_logger(), "Color→AUTO"); }
        else if (key == 'h') { show_heatmap_ = !show_heatmap_;
                               RCLCPP_INFO(get_logger(), "Heatmap %s",
                                   show_heatmap_ ? "ON" : "OFF"); }
        else if (key == 'H') {
            memset(enemy_heat_, 0, sizeof(enemy_heat_));
            memset(ally_heat_,  0, sizeof(ally_heat_));
            heatmap_overlay_ = cv::Mat();  // force re-render
            RCLCPP_INFO(get_logger(), "Heatmap cleared");
        }
        else if (key == 'v') {
            double_vuln_triggered_++;
            auto m = std_msgs::msg::UInt8(); m.data = double_vuln_triggered_;
            decision_pub_->publish(m);
            RCLCPP_WARN(get_logger(), "Manual vuln #%d", double_vuln_triggered_);
        }

        cv::imshow("RM2026 Radar", canvas);
        publish_debug_points(time);

        // Publish map to /map_2d for RViz/rqt
        if (debug_map_pub->get_subscription_count() > 0) {
            auto img_msg = std::make_shared<sensor_msgs::msg::Image>();
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", map_roi)
                .toImageMsg(*img_msg);
            debug_map_pub->publish(*img_msg);
        }
    }

    void publish_debug_points(double time)
    {
        if (debug_points_pub->get_subscription_count() == 0) return;
        vision_interface::msg::DetectResult msg;
        msg.header.stamp = this->get_clock()->now();
        for (int i = 0; i < kRobotSlotCount; i++) {
            const double blue_age = time - blue_time[i];
            if (blue_point[i].x > 0.1F && blue_point[i].y > 0.1F &&
                blue_age < 30.0 && blue_time[i] > 0.0) {
                msg.blue_x[i] = blue_point[i].x;
                msg.blue_y[i] = blue_point[i].y;
            }
            const double red_age = time - red_time[i];
            if (red_point[i].x > 0.1F && red_point[i].y > 0.1F &&
                red_age < 30.0 && red_time[i] > 0.0) {
                msg.red_x[i] = red_point[i].x;
                msg.red_y[i] = red_point[i].y;
            }
        }
        debug_points_pub->publish(msg);
    }

    void camera_callback(
        const std::shared_ptr<vision_interface::msg::DetectResult> msg)
    {
        auto   now = std::chrono::system_clock::now();
        double time = std::chrono::duration_cast<std::chrono::milliseconds>(
                          now.time_since_epoch()).count() / 1000.0;
        static constexpr double kKalmanFreshTimeout = 0.18;
        // Adaptive EMA: strong smoothing for small jitter (partial occlusion),
        // near-raw for large deltas (real movement) → no trailing/lag.
        static constexpr double kEmaStaleS   = 1.0;    // reset if gap > 1 s
        static constexpr float  kEmaJumpM    = 3.0F;   // snap if jump > 3 m
        static constexpr float  kEmaRampM    = 1.5F;   // delta at which alpha→max
        static constexpr float  kEmaAlphaMin = 0.3F;   // strong smoothing (jitter)
        static constexpr float  kEmaAlphaMax = 0.9F;   // nearly raw (movement)

        auto ema_smooth = [&](cv::Point2f& prev, double prev_time,
                              const cv::Point2f& raw) -> cv::Point2f {
            if (prev_time > 0 && (time - prev_time) < kEmaStaleS) {
                const float dx = raw.x - prev.x, dy = raw.y - prev.y;
                const float dist = std::sqrt(dx * dx + dy * dy);
                if (dist > kEmaJumpM) return raw;  // snap on large jump
                const float alpha = std::clamp(
                    dist / kEmaRampM, kEmaAlphaMin, kEmaAlphaMax);
                return cv::Point2f(
                    alpha * raw.x + (1.0F - alpha) * prev.x,
                    alpha * raw.y + (1.0F - alpha) * prev.y);
            }
            return raw;  // snap: first detection or stale
        };

        // Camera-primary minimap: 3D-box-validated camera positions always
        // win. KF tracking from LiDAR cluster (which over-segments field
        // structures into ghost robots) only fills in when camera is silent.
        for (int i = 0; i < kRobotSlotCount; i++) {
            if (msg->blue_x[i] * msg->blue_y[i]) {
                blue_camera_time[i] = time;
                cv::Point2f raw = normalizeToGlobal(
                    cv::Point2f(msg->blue_x[i], msg->blue_y[i]));
                blue_point[i] = ema_smooth(blue_point[i], blue_time[i], raw);
                blue_time[i] = time;
                blue_update[i] = time;
                blue_vel[i] = cv::Point2f(0.0F, 0.0F);
                blue_behavior[i] = 0;
            }
            if (msg->red_x[i] * msg->red_y[i]) {
                red_camera_time[i] = time;
                cv::Point2f raw = normalizeToGlobal(
                    cv::Point2f(msg->red_x[i], msg->red_y[i]));
                red_point[i] = ema_smooth(red_point[i], red_time[i], raw);
                red_time[i] = time;
                red_update[i] = time;
                red_vel[i] = cv::Point2f(0.0F, 0.0F);
                red_behavior[i] = 0;
            }
        }
    }

    void
    callback(const std::shared_ptr<vision_interface::msg::DetectResult> msg)
    {
        auto   now = std::chrono::system_clock::now();
        double time = std::chrono::duration_cast<std::chrono::milliseconds>(
                          now.time_since_epoch()).count() / 1000.0;
        for (int i = 0; i < kRobotSlotCount; i++) {
            if (msg->blue_x[i] * msg->blue_y[i]) {
                const cv::Point2f new_pt = normalizeToGlobal(
                    cv::Point2f(msg->blue_x[i], msg->blue_y[i]));
                const double dt = time - blue_time[i];
                if (dt > 0.01 && dt < 2.0 && blue_time[i] > 0)
                    blue_vel[i] = (new_pt - blue_point[i]) * static_cast<float>(1.0 / dt);
                // Infer behavior from position+velocity (matches kalman_filter logic)
                blue_behavior[i] = 0;
                if (new_pt.x > arena_width_ - 7.5F && blue_vel[i].x > 0.8F) blue_behavior[i] = 1;
                if (new_pt.x < 7.5F               && blue_vel[i].x < -0.8F) blue_behavior[i] = 1;
                // TUNNEL: x=8-12 or x=16-20, y=3-4.2 or y=10.8-12
                if (((new_pt.x > 7.5F && new_pt.x < 12.5F) || (new_pt.x > 15.5F && new_pt.x < 20.5F)) &&
                    ((new_pt.y > 2.5F && new_pt.y < 4.5F)  || (new_pt.y > 10.5F && new_pt.y < 12.5F)))
                    blue_behavior[i] = 2;  // TUNNEL
                // ASSEMBLY: engineer(i==1) leaving base heading toward center (trajectory-based)
                if (i == 1 &&
                    (((new_pt.x > 3.0F && new_pt.x < 16.5F) && blue_vel[i].x > 0.3F) ||
                     ((new_pt.x > 11.5F && new_pt.x < 25.0F) && blue_vel[i].x < -0.3F)) &&
                    (new_pt.y > 3.0F && new_pt.y < 12.0F))
                    blue_behavior[i] = 4;  // ASSEMBLY
                // OUTPOST_AMBUSH: 蓝英雄停在 cross_tower 一侧 (x=8-16, 自家前哨附近)
                const float bspd2 = blue_vel[i].x*blue_vel[i].x + blue_vel[i].y*blue_vel[i].y;
                if (i == 0 && bspd2 < 2.25F &&
                    new_pt.x > 8.0F && new_pt.x < 16.0F &&
                    new_pt.y > 4.0F && new_pt.y < 11.0F)
                    blue_behavior[i] = 3;  // OUTPOST_AMBUSH
                blue_point[i]  = new_pt;
                blue_time[i]   = time;
                blue_kalman_time[i] = time;
                blue_update[i] = time;
            }
            if (msg->red_x[i] * msg->red_y[i]) {
                const cv::Point2f new_pt = normalizeToGlobal(
                    cv::Point2f(msg->red_x[i], msg->red_y[i]));
                const double dt = time - red_time[i];
                if (dt > 0.01 && dt < 2.0 && red_time[i] > 0)
                    red_vel[i] = (new_pt - red_point[i]) * static_cast<float>(1.0 / dt);
                red_behavior[i] = 0;
                if (new_pt.x > arena_width_ - 7.5F && red_vel[i].x > 0.8F) red_behavior[i] = 1;
                if (new_pt.x < 7.5F               && red_vel[i].x < -0.8F) red_behavior[i] = 1;
                // TUNNEL: x=8-12 or x=16-20, y=3-4.2 or y=10.8-12
                if (((new_pt.x > 7.5F && new_pt.x < 12.5F) || (new_pt.x > 15.5F && new_pt.x < 20.5F)) &&
                    ((new_pt.y > 2.5F && new_pt.y < 4.5F)  || (new_pt.y > 10.5F && new_pt.y < 12.5F)))
                    red_behavior[i] = 2;  // TUNNEL
                // ASSEMBLY: engineer(i==1) leaving base heading toward center (trajectory-based)
                if (i == 1 &&
                    (((new_pt.x > 3.0F && new_pt.x < 16.5F) && red_vel[i].x > 0.3F) ||
                     ((new_pt.x > 11.5F && new_pt.x < 25.0F) && red_vel[i].x < -0.3F)) &&
                    (new_pt.y > 3.0F && new_pt.y < 12.0F))
                    red_behavior[i] = 4;  // ASSEMBLY
                // OUTPOST_AMBUSH: 红英雄停在 self_tower 一侧 (x=12-20, 自家前哨附近)
                const float rspd2 = red_vel[i].x*red_vel[i].x + red_vel[i].y*red_vel[i].y;
                if (i == 0 && rspd2 < 2.25F &&
                    new_pt.x > 12.0F && new_pt.x < 20.0F &&
                    new_pt.y > 4.0F && new_pt.y < 11.0F)
                    red_behavior[i] = 3;
                red_point[i]  = new_pt;
                red_time[i]   = time;
                red_kalman_time[i] = time;
                red_update[i] = time;
            }
        }
        // Heatmap accumulation (every frame)
        {
            const int self_c = resolveSelfColor();
            auto accum = [&](float (&grid)[30][56], const cv::Point2f& pt) {
                const int ix = std::clamp(static_cast<int>(pt.x / kHR), 0, kHX - 1);
                const int iy = std::clamp(static_cast<int>(pt.y / kHR), 0, kHY - 1);
                grid[iy][ix] += 1.0F;
            };
            for (int i = 0; i < kRobotSlotCount; i++) {
                if (blue_point[i].x > 0.1F && blue_time[i] == time) {
                    if (self_c == 0) accum(ally_heat_,  blue_point[i]);
                    else             accum(enemy_heat_, blue_point[i]);
                }
                if (red_point[i].x > 0.1F && red_time[i] == time) {
                    if (self_c == 2) accum(ally_heat_,  red_point[i]);
                    else             accum(enemy_heat_, red_point[i]);
                }
            }
        }
        // /radar_warn dead-publish 已移除: 仅 debug_map 自己写, 全栈无订阅者。
        //   - dart/fly 警告已在 UI overlay 直接渲染 (见 "!! DART !!" / "UAV:*" putText)。
        //   - hero_state 派生不再有任何消费者; 决策端 (kalman_filter / radar_serial_node)
        //     直接用 KF 跟踪结果与 marks, 不依赖此粗略 1Hz 计数。
        // /Radar2Sentry 路径同样已废弃: 0x0305 直接来自 kalman_filter 的 /radar2sentry。

        // Auto-trigger double vulnerability: use it as soon as available
        uint8_t opportunities = match_info.ultimate & 0x03U;
        bool vuln_active = (match_info.ultimate >> 2U) & 0x01U;
        bool any_marked = false;
        for (int i = 0; i < kRobotSlotCount; i++) {
            if (match_info.marks[i] >= 1) { any_marked = true; break; }
        }
        // 判断是否应触发
        // 全自动: opportunities > triggered + 至少一个有效标记 + 非激活态 → 立即触发
        const bool should_trigger = (opportunities > double_vuln_triggered_)
                                    && any_marked && !vuln_active;
        if (should_trigger) {
            double_vuln_triggered_++;
            auto msg = std_msgs::msg::UInt8();
            msg.data = double_vuln_triggered_;
            decision_pub_->publish(msg);
            RCLCPP_WARN(this->get_logger(),
                "[AUTO] Triggered double vuln #%d (match_time=%d)",
                double_vuln_triggered_, match_info.match_time);
        }
    }
    rclcpp::Subscription<vision_interface::msg::DetectResult>::SharedPtr
        detect_result_sub;
    rclcpp::Subscription<vision_interface::msg::DetectResult>::SharedPtr
        camera_detect_sub;
    rclcpp::Subscription<vision_interface::msg::RadarWarn>::SharedPtr
        lidar_warn_sub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_map_pub;
    rclcpp::Publisher<vision_interface::msg::DetectResult>::SharedPtr debug_points_pub;
    rclcpp::Subscription<vision_interface::msg::MatchInfo>::SharedPtr
        match_info_sub;
    rclcpp::TimerBase::SharedPtr ui_timer_;

    double blue_time[kRobotSlotCount];
    double red_time[kRobotSlotCount];
    double blue_kalman_time[kRobotSlotCount];
    double red_kalman_time[kRobotSlotCount];
    double blue_camera_time[kRobotSlotCount];
    double red_camera_time[kRobotSlotCount];

    double blue_update[kRobotSlotCount];
    double red_update[kRobotSlotCount];
    cv::Point2f blue_vel[kRobotSlotCount];
    cv::Point2f red_vel[kRobotSlotCount];
    uint8_t blue_behavior[kRobotSlotCount] = {};
    uint8_t red_behavior[kRobotSlotCount]  = {};

    uint8_t double_vuln_triggered_ = 0;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr decision_pub_;

    cv::Point2f blue_point[kRobotSlotCount];
    cv::Point2f red_point[kRobotSlotCount];

    // ── 位置热力图 (0.5m分辨率, 56×30格) ───────────────────────────────────
    // 用途: 比较己方与敌方热力图, 缺失区域≈遮蔽盲区
    // 键盘: h=切换显示, H=清除积累
    static constexpr int   kHX = 56;    // 28m / 0.5m
    static constexpr int   kHY = 30;    // 15m / 0.5m
    static constexpr float kHR = 0.5F;  // 格子分辨率(m)
    float enemy_heat_[30][56] = {};
    float ally_heat_[30][56]  = {};
    bool  show_heatmap_       = false;
    cv::Mat heatmap_overlay_;           // 预渲染结果, 按需刷新
    double  heatmap_dirty_t_  = 0.0;    // 上次渲染时间

    vision_interface::msg::MatchInfo match_info;
    vision_interface::msg::RadarWarn lidar_warn_;
    cv::Mat                          map;
    int                              count = 0;
    std::string                      map_image_path_ = "config/map/map.jpg";
    float                            arena_width_ = 28.0F;
    float                            arena_height_ = 15.0F;
    bool                             input_is_self_frame_ = false;
    int                              self_color_override_ = -1;
};
}  // namespace tdt_radar

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node_options = rclcpp::NodeOptions();
    rclcpp::spin(std::make_shared<tdt_radar::DebugMap>(node_options));
    rclcpp::shutdown();
    return 0;
}
