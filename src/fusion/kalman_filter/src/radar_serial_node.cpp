#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <vision_interface/msg/detect_result.hpp>
#include <vision_interface/msg/match_info.hpp>
#include <vision_interface/msg/radar2_sentry.hpp>
#include "kalman_filter.h"

namespace tdt_radar {

class RadarSerialNode : public rclcpp::Node {
public:
  RadarSerialNode() : Node("radar_serial_node") {
    match_info_cache_.match_time = -200;
    node_start_sec_ = this->get_clock()->now().seconds();
    port_ = this->declare_parameter<std::string>("port", "/dev/gimbal");
    baud_rate_ = this->declare_parameter<int>("baud_rate", 115200);
    send_hz_ = this->declare_parameter<double>("send_hz", 5.0);
    receive_hz_ = this->declare_parameter<double>("receive_hz", 50.0);
    coord_scale_ = this->declare_parameter<double>("coord_scale", 100.0);
    map_width_ = this->declare_parameter<double>("map_width", 28.0);
    map_height_ = this->declare_parameter<double>("map_height", 15.0);
    tx_cmd_id_ = static_cast<uint16_t>(this->declare_parameter<int>("tx_cmd_id", 0x0305));
    radar_topic_ = this->declare_parameter<std::string>("radar_topic", "/radar2sentry");
    radar_topic_compat_ =
      this->declare_parameter<std::string>("radar_topic_compat", "/Radar2Sentry");
    enable_resolve_fallback_ =
      this->declare_parameter<bool>("enable_resolve_fallback", false);
    resolve_topic_ =
      this->declare_parameter<std::string>("resolve_topic", "/resolve_result");
    kalman_topic_ =
      this->declare_parameter<std::string>("kalman_topic", "/kalman_detect");
    map_points_topic_ =
      this->declare_parameter<std::string>("map_points_topic", "/debug_map_points");
    radar_topic_timeout_s_ =
      this->declare_parameter<double>("radar_topic_timeout_s", 0.5);
    fallback_slot_hold_s_ =
      this->declare_parameter<double>("fallback_slot_hold_s", 30.0);
    send_match_topic_ = this->declare_parameter<std::string>("match_info_topic", "/match_info");
    self_color_override_ = this->declare_parameter<int>("self_color_override", -1);

    enable_decision_ = this->declare_parameter<bool>("enable_decision", true);
    decision_topic_ = this->declare_parameter<std::string>("decision_topic", "/radar/decision_request");
    decision_receiver_id_ =
      static_cast<uint16_t>(this->declare_parameter<int>("decision_receiver_id", 0x8080));
    decision_content_id_ =
      static_cast<uint16_t>(this->declare_parameter<int>("decision_content_id", 0x0121));
    decision_min_interval_s_ = this->declare_parameter<double>("decision_min_interval_s", 0.10);
    publish_match_info_ = this->declare_parameter<bool>("publish_match_info", true);
    match_info_timeout_s_ = this->declare_parameter<double>("match_info_timeout_s", 2.0);

    rx_expect_cmd_id_ =
        static_cast<uint16_t>(this->declare_parameter<int>("rx_expect_cmd_id", 0x0303));
    enable_receive_ = this->declare_parameter<bool>("enable_receive", true);
    log_rx_payload_hex_ = this->declare_parameter<bool>("log_rx_payload_hex", false);
    dry_run_ = this->declare_parameter<bool>("dry_run", false);

    match_sub_ = this->create_subscription<vision_interface::msg::MatchInfo>(
      send_match_topic_, rclcpp::SensorDataQoS(),
      std::bind(&RadarSerialNode::onMatchInfo, this, std::placeholders::_1));

    radar_sub_ = this->create_subscription<vision_interface::msg::Radar2Sentry>(
        radar_topic_, rclcpp::SensorDataQoS(),
        std::bind(&RadarSerialNode::onRadar, this, std::placeholders::_1));
    if (radar_topic_compat_ != radar_topic_) {
      radar_sub_compat_ = this->create_subscription<vision_interface::msg::Radar2Sentry>(
          radar_topic_compat_, rclcpp::SensorDataQoS(),
          std::bind(&RadarSerialNode::onRadar, this, std::placeholders::_1));
    }
    if (enable_resolve_fallback_) {
      map_points_sub_ = this->create_subscription<vision_interface::msg::DetectResult>(
          map_points_topic_, rclcpp::SensorDataQoS(),
          std::bind(&RadarSerialNode::onMapPoints, this, std::placeholders::_1));
      kalman_sub_ = this->create_subscription<vision_interface::msg::DetectResult>(
          kalman_topic_, rclcpp::SensorDataQoS(),
          std::bind(&RadarSerialNode::onKalmanDetect, this, std::placeholders::_1));
      resolve_sub_ = this->create_subscription<vision_interface::msg::DetectResult>(
          resolve_topic_, rclcpp::SensorDataQoS(),
          std::bind(&RadarSerialNode::onResolve, this, std::placeholders::_1));
    }

    if (enable_decision_) {
      decision_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        decision_topic_, 10, std::bind(&RadarSerialNode::onDecisionRequest, this,
                       std::placeholders::_1));
    }
    // /lidar_detect (dart/fly) 与 /radar_warn 均不进串口、不向哨兵转发, 故无订阅。

    rx_0303_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/radar/rx_0303_raw", 10);
    rx_raw_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/radar/rx_raw", 10);
    tx_raw_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/radar/tx_raw", 10);
    if (publish_match_info_) {
      match_info_pub_ = this->create_publisher<vision_interface::msg::MatchInfo>("/match_info", 10);
    }

    if (send_hz_ <= 0.0) {
      send_hz_ = 5.0;
      RCLCPP_WARN(this->get_logger(), "send_hz <= 0, fallback to 5.0");
    }
    if (tx_cmd_id_ == 0x0305U && send_hz_ > 5.0) {
      send_hz_ = 5.0;
      RCLCPP_WARN(this->get_logger(),
                  "0x0305 map robot data is limited to 5Hz by protocol, clamp send_hz to 5.0");
    }

    if (receive_hz_ <= 0.0) {
      receive_hz_ = 50.0;
      RCLCPP_WARN(this->get_logger(), "receive_hz <= 0, fallback to 50.0");
    }
    if (map_width_ <= 0.0) {
      map_width_ = 28.0;
      RCLCPP_WARN(this->get_logger(), "map_width <= 0, fallback to 28.0");
    }
    if (map_height_ <= 0.0) {
      map_height_ = 15.0;
      RCLCPP_WARN(this->get_logger(), "map_height <= 0, fallback to 15.0");
    }

    send_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / send_hz_)),
        std::bind(&RadarSerialNode::onSendTimer, this));

    if (enable_receive_) {
      recv_timer_ = this->create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::duration<double>(1.0 / receive_hz_)),
          std::bind(&RadarSerialNode::onRecvTimer, this));
    }

    if (!dry_run_) {
      tryOpenSerial();
    }

    if (radar_topic_timeout_s_ <= 0.0) {
      radar_topic_timeout_s_ = 0.5;
      RCLCPP_WARN(this->get_logger(), "radar_topic_timeout_s <= 0, fallback to 0.5");
    }
    if (fallback_slot_hold_s_ < radar_topic_timeout_s_) {
      fallback_slot_hold_s_ = radar_topic_timeout_s_;
      RCLCPP_WARN(this->get_logger(),
                  "fallback_slot_hold_s < radar_topic_timeout_s, fallback to %.2f",
                  fallback_slot_hold_s_);
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Radar serial node started. port=%s baud=%d tx_cmd=0x%04X radar_topic=%s compat_topic=%s "
        "resolve_fallback=%s resolve_topic=%s kalman_topic=%s map_points_topic=%s timeout=%.2fs slot_hold=%.2fs map=(%.2f, %.2f)m payload=48B(enemy+self)",
        port_.c_str(), baud_rate_, tx_cmd_id_, radar_topic_.c_str(),
        radar_topic_compat_.c_str(), enable_resolve_fallback_ ? "true" : "false",
        resolve_topic_.c_str(), kalman_topic_.c_str(), map_points_topic_.c_str(),
        radar_topic_timeout_s_, fallback_slot_hold_s_, map_width_, map_height_);
  }

  ~RadarSerialNode() override {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

private:
  static uint8_t crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; ++i) {
      crc ^= data[i];
      for (int bit = 0; bit < 8; ++bit) {
        if (crc & 0x01U) {
          crc = static_cast<uint8_t>((crc >> 1U) ^ 0x8CU);
        } else {
          crc = static_cast<uint8_t>(crc >> 1U);
        }
      }
    }
    return crc;
  }

  static uint16_t crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
      crc ^= data[i];
      for (int bit = 0; bit < 8; ++bit) {
        if (crc & 0x0001U) {
          crc = static_cast<uint16_t>((crc >> 1U) ^ 0x8408U);
        } else {
          crc = static_cast<uint16_t>(crc >> 1U);
        }
      }
    }
    return crc;
  }

  static std::string bytesToHex(const uint8_t *data, size_t len) {
    static constexpr char kHex[] = "0123456789ABCDEF";
    std::string out;
    out.reserve(len * 3);
    for (size_t i = 0; i < len; ++i) {
      const uint8_t v = data[i];
      out.push_back(kHex[v >> 4]);
      out.push_back(kHex[v & 0x0F]);
      if (i + 1 != len) {
        out.push_back(' ');
      }
    }
    return out;
  }

  static void appendU16LE(std::vector<uint8_t> &buf, uint16_t v) {
    buf.push_back(static_cast<uint8_t>(v & 0xFFU));
    buf.push_back(static_cast<uint8_t>((v >> 8U) & 0xFFU));
  }

  static bool hasValidCoord(float x, float y) {
    return std::isfinite(x) && std::isfinite(y) && x > 0.0F && y > 0.0F;
  }

  static const char *slotName(size_t idx) {
    static constexpr const char *kNames[kRobotSlotCount] = {
        "hero", "engineer", "infantry3", "infantry4", "sentry"};
    return idx < kRobotSlotCount ? kNames[idx] : "unknown";
  }

  // Protocol 0x0305/0x020C position 0..5 = robot 1/2/3/4/6(aerial)/7
  static constexpr int kProtoSlotCount = 6;
  static constexpr int kProtoPosToSlot[kProtoSlotCount] = {0, 1, 2, 3, -1, 4};
  static constexpr int kSlotToProtoPos[kRobotSlotCount] = {0, 1, 2, 3, 5};

  bool tryOpenSerial() {
    if (fd_ >= 0) {
      return true;
    }

    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Open serial failed: %s (%s)", port_.c_str(), std::strerror(errno));
      return false;
    }

    termios tty {};
    if (tcgetattr(fd_, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcgetattr failed: %s", std::strerror(errno));
      ::close(fd_);
      fd_ = -1;
      return false;
    }

    const speed_t baud = toSpeed(baud_rate_);
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    tcflush(fd_, TCIFLUSH);
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcsetattr failed: %s", std::strerror(errno));
      ::close(fd_);
      fd_ = -1;
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Serial opened: %s @ %d", port_.c_str(), baud_rate_);
    return true;
  }

  static speed_t toSpeed(int baud_rate) {
    switch (baud_rate) {
      case 9600:
        return B9600;
      case 19200:
        return B19200;
      case 38400:
        return B38400;
      case 57600:
        return B57600;
      case 115200:
        return B115200;
      case 230400:
        return B230400;
      case 460800:
        return B460800;
      default:
        return B115200;
    }
  }

  void closeSerialWithWarn(const char *reason) {
    if (fd_ >= 0) {
      RCLCPP_WARN(this->get_logger(), "%s, close serial fd", reason);
      ::close(fd_);
      fd_ = -1;
    }
  }

  void onMatchInfo(const vision_interface::msg::MatchInfo::SharedPtr msg) {
    match_info_ = *msg;
    has_match_info_ = true;
  }

  void onRadar(const vision_interface::msg::Radar2Sentry::SharedPtr msg) {
    latest_radar_direct_ = *msg;
    has_radar_direct_ = true;
    last_radar_direct_sec_ = this->get_clock()->now().seconds();
  }

  void updateFallbackFromDetect(const vision_interface::msg::DetectResult &msg) {
    if (!enable_resolve_fallback_) {
      return;
    }

    const int self_color = resolveSelfColor();
    if (self_color != 0 && self_color != 2) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "self_color unknown, skip /resolve_result fallback");
      return;
    }

    vision_interface::msg::Radar2Sentry radar_msg = latest_radar_fallback_;
    const double now = this->get_clock()->now().seconds();
    auto update_slot = [&](float in_x, float in_y, float &out_x, float &out_y,
                           double &slot_time) {
      if (hasValidCoord(in_x, in_y)) {
        out_x = in_x;
        out_y = in_y;
        slot_time = now;
        return;
      }
      if ((now - slot_time) > fallback_slot_hold_s_) {
        out_x = 0.0F;
        out_y = 0.0F;
      }
    };
    for (size_t i = 0; i < kRobotSlotCount; ++i) {
      if (self_color == 0) {
        update_slot(msg.red_x[i], msg.red_y[i], radar_msg.radar_enemy_x[i],
                    radar_msg.radar_enemy_y[i], fallback_enemy_slot_sec_[i]);
        update_slot(msg.blue_x[i], msg.blue_y[i], radar_msg.radar_self_x[i],
                    radar_msg.radar_self_y[i], fallback_self_slot_sec_[i]);
      } else {
        update_slot(msg.blue_x[i], msg.blue_y[i], radar_msg.radar_enemy_x[i],
                    radar_msg.radar_enemy_y[i], fallback_enemy_slot_sec_[i]);
        update_slot(msg.red_x[i], msg.red_y[i], radar_msg.radar_self_x[i],
                    radar_msg.radar_self_y[i], fallback_self_slot_sec_[i]);
      }
    }

    latest_radar_fallback_ = radar_msg;
    has_radar_fallback_ = true;
    last_radar_fallback_sec_ = now;
    if (countActiveEnemies(radar_msg) > 0) {
      latest_nonzero_radar_fallback_ = radar_msg;
      has_nonzero_radar_fallback_ = true;
      last_nonzero_radar_fallback_sec_ = last_radar_fallback_sec_;
    }
  }

  void onKalmanDetect(const vision_interface::msg::DetectResult::SharedPtr msg) {
    updateFallbackFromDetect(*msg);
  }

  void onMapPoints(const vision_interface::msg::DetectResult::SharedPtr msg) {
    updateFallbackFromDetect(*msg);
  }

  void onResolve(const vision_interface::msg::DetectResult::SharedPtr msg) {
    updateFallbackFromDetect(*msg);
  }

  void onDecisionRequest(const std_msgs::msg::UInt8::SharedPtr msg) {
    pending_decision_byte_ = msg->data;
    has_pending_decision_ = true;
  }

  bool selectRadarForTx(vision_interface::msg::Radar2Sentry &radar_out,
                        bool &using_fallback) const {
    using_fallback = false;
    const double now = this->get_clock()->now().seconds();

    auto merge_valid_slots = [](vision_interface::msg::Radar2Sentry &dst,
                                const vision_interface::msg::Radar2Sentry &src) {
      for (size_t i = 0; i < kRobotSlotCount; ++i) {
        if (hasValidCoord(src.radar_enemy_x[i], src.radar_enemy_y[i])) {
          dst.radar_enemy_x[i] = src.radar_enemy_x[i];
          dst.radar_enemy_y[i] = src.radar_enemy_y[i];
        }
        if (hasValidCoord(src.radar_self_x[i], src.radar_self_y[i])) {
          dst.radar_self_x[i] = src.radar_self_x[i];
          dst.radar_self_y[i] = src.radar_self_y[i];
        }
      }
    };

    const bool direct_fresh =
        has_radar_direct_ && (now - last_radar_direct_sec_) <= radar_topic_timeout_s_;
    const bool fallback_fresh =
        enable_resolve_fallback_ && has_radar_fallback_ &&
        (now - last_radar_fallback_sec_) <= radar_topic_timeout_s_;

    if (direct_fresh || fallback_fresh) {
      vision_interface::msg::Radar2Sentry merged;
      if (direct_fresh) {
        merged = latest_radar_direct_;
      }
      if (fallback_fresh) {
        merge_valid_slots(merged, latest_radar_fallback_);
        using_fallback = true;
      }
      if (hasAnyActiveCoord(merged)) {
        radar_out = merged;
        return true;
      }
    }

    if (enable_resolve_fallback_ && has_radar_fallback_) {
      if (hasAnyActiveCoord(latest_radar_fallback_)) {
        radar_out = latest_radar_fallback_;
        using_fallback = true;
        return true;
      }
      if (has_nonzero_radar_fallback_ && hasAnyActiveCoord(latest_nonzero_radar_fallback_)) {
        radar_out = latest_nonzero_radar_fallback_;
        using_fallback = true;
        return true;
      }
    }

    if (has_radar_direct_) {
      if (hasAnyActiveCoord(latest_radar_direct_)) {
        radar_out = latest_radar_direct_;
        return true;
      }
    }

    if (enable_resolve_fallback_ && has_radar_fallback_) {
      const bool fallback_fresh = (now - last_radar_fallback_sec_) <= radar_topic_timeout_s_;
      if (fallback_fresh && hasAnyActiveCoord(latest_radar_fallback_)) {
        radar_out = latest_radar_fallback_;
        using_fallback = true;
        return true;
      }
    }

    return false;
  }

  float clampCoordForMap(float v, double limit) const {
    if (!std::isfinite(v) || v <= 0.0F) {
      return 0.0F;
    }
    const double step = 1.0 / std::max(coord_scale_, 1.0);
    const double min_inside = std::min(step, limit);
    const double max_inside = std::max(limit - step, min_inside);
    return static_cast<float>(
        std::clamp(static_cast<double>(v), min_inside, max_inside));
  }

  std::vector<uint8_t> buildRadarPayload(
      const vision_interface::msg::Radar2Sentry &radar_msg) const {
    // 0x0305: 6 enemy + 6 ally (uint16 x/y cm). aerial slot=0.
    std::vector<uint8_t> payload;
    payload.reserve(48);
    auto write_side = [&](const float *xs, const float *ys) {
      for (int pos = 0; pos < kProtoSlotCount; ++pos) {
        const int slot = kProtoPosToSlot[pos];
        const float x = (slot >= 0) ? xs[slot] : 0.0F;
        const float y = (slot >= 0) ? ys[slot] : 0.0F;
        appendU16LE(payload, encodeCoord(clampCoordForMap(x, map_width_)));
        appendU16LE(payload, encodeCoord(clampCoordForMap(y, map_height_)));
      }
    };
    write_side(radar_msg.radar_enemy_x.data(), radar_msg.radar_enemy_y.data());
    write_side(radar_msg.radar_self_x.data(),  radar_msg.radar_self_y.data());
    return payload;
  }

  size_t countActiveEnemies(const vision_interface::msg::Radar2Sentry &radar_msg) const {
    size_t active = 0;
    for (size_t i = 0; i < kRobotSlotCount; ++i) {
      if (hasValidCoord(radar_msg.radar_enemy_x[i], radar_msg.radar_enemy_y[i])) {
        ++active;
      }
    }
    return active;
  }

  bool hasAnyActiveCoord(const vision_interface::msg::Radar2Sentry &radar_msg) const {
    for (size_t i = 0; i < kRobotSlotCount; ++i) {
      if (hasValidCoord(radar_msg.radar_enemy_x[i], radar_msg.radar_enemy_y[i]) ||
          hasValidCoord(radar_msg.radar_self_x[i], radar_msg.radar_self_y[i])) {
        return true;
      }
    }
    return false;
  }

  bool firstActiveEnemy(const vision_interface::msg::Radar2Sentry &radar_msg, float &x,
                        float &y, size_t &slot_idx) const {
    for (size_t i = 0; i < kRobotSlotCount; ++i) {
      if (hasValidCoord(radar_msg.radar_enemy_x[i], radar_msg.radar_enemy_y[i])) {
        x = radar_msg.radar_enemy_x[i];
        y = radar_msg.radar_enemy_y[i];
        slot_idx = i;
        return true;
      }
    }
    x = 0.0F;
    y = 0.0F;
    slot_idx = 0;
    return false;
  }

  void logTxSummary(const vision_interface::msg::Radar2Sentry &radar_msg, bool using_fallback) {
    float first_x = 0.0F;
    float first_y = 0.0F;
    size_t first_slot_idx = 0;
    const bool has_first = firstActiveEnemy(radar_msg, first_x, first_y, first_slot_idx);
    const size_t active_enemies = countActiveEnemies(radar_msg);

    if (has_first) {
      const float tx_x = clampCoordForMap(first_x, map_width_);
      const float tx_y = clampCoordForMap(first_y, map_height_);
      RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 3000,
          "TX radar packet source=%s active_enemies=%zu first_enemy_robot=%d(%s) first_enemy=(%.2f, %.2f)",
          using_fallback ? "detect_cache" : "radar_topic", active_enemies,
          slot_to_robot_number(static_cast<int>(first_slot_idx)),
          slotName(first_slot_idx), tx_x, tx_y);
      return;
    }

    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "TX radar packet source=%s has no active enemy coordinates; payload will be all zeros",
        using_fallback ? "detect_cache" : "radar_topic");
  }

  void logTxSlots(const vision_interface::msg::Radar2Sentry &radar_msg, bool using_fallback) {
    const double now = this->get_clock()->now().seconds();
    if (now - last_tx_slots_log_sec_ < 1.0) {
      return;
    }
    last_tx_slots_log_sec_ = now;

    auto fmt_side = [&](const float *xs, const float *ys) {
      std::string out;
      for (int pos = 0; pos < kProtoSlotCount; ++pos) {
        const int slot = kProtoPosToSlot[pos];
        const int robot_no = (pos == 4) ? 6 : slot_to_robot_number(slot);
        const float x = (slot >= 0) ? clampCoordForMap(xs[slot], map_width_) : 0.0F;
        const float y = (slot >= 0) ? clampCoordForMap(ys[slot], map_height_) : 0.0F;
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%s%d=(%.2f,%.2f)", pos == 0 ? "" : " ",
                      robot_no, x, y);
        out += buf;
      }
      return out;
    };

    const std::string enemy =
        fmt_side(radar_msg.radar_enemy_x.data(), radar_msg.radar_enemy_y.data());
    const std::string self =
        fmt_side(radar_msg.radar_self_x.data(), radar_msg.radar_self_y.data());
    RCLCPP_INFO(this->get_logger(),
                "TX0305 slots source=%s enemy[%s] self[%s]",
                using_fallback ? "detect_cache" : "radar_topic", enemy.c_str(),
                self.c_str());
  }

  uint16_t encodeCoord(float v) const {
    if (!std::isfinite(v) || v <= 0.0f) {
      return 0;
    }
    const double scaled = static_cast<double>(v) * coord_scale_;
    if (scaled <= 0.0) {
      return 0;
    }
    const double max_u16 = static_cast<double>(std::numeric_limits<uint16_t>::max());
    const double clamped = std::min(scaled, max_u16);
    return static_cast<uint16_t>(std::lround(clamped));
  }

  std::vector<uint8_t> buildPacket(const std::vector<uint8_t> &payload, uint16_t cmd_id) {
    std::vector<uint8_t> packet;
    packet.reserve(5 + 2 + payload.size() + 2);

    packet.push_back(0xA5);
    appendU16LE(packet, static_cast<uint16_t>(payload.size()));
    packet.push_back(seq_);
    packet.push_back(crc8(packet.data(), 4));

    appendU16LE(packet, cmd_id);
    packet.insert(packet.end(), payload.begin(), payload.end());

    const uint16_t crc = crc16(packet.data(), packet.size());
    appendU16LE(packet, crc);

    seq_ = static_cast<uint8_t>((seq_ + 1U) & 0xFFU);
    return packet;
  }

  int resolveSelfColor() const {
    if (self_color_override_ == 0 || self_color_override_ == 2) {
      return self_color_override_;
    }

    if (has_match_info_ && (match_info_.self_color == 0 || match_info_.self_color == 2)) {
      return match_info_.self_color;
    }

    return -1;
  }

  uint16_t resolveRadarRobotId() const {
    const int self_color = resolveSelfColor();
    if (self_color == 0) {
      return 109U;
    }
    if (self_color == 2) {
      return 9U;
    }
    return 0U;
  }

  std::vector<uint8_t> buildRobotInteractionPayload(uint16_t content_id, uint16_t sender_id,
                                                    uint16_t receiver_id,
                                                    const std::vector<uint8_t> &content) const {
    std::vector<uint8_t> payload;
    payload.reserve(6 + content.size());
    appendU16LE(payload, content_id);
    appendU16LE(payload, sender_id);
    appendU16LE(payload, receiver_id);
    payload.insert(payload.end(), content.begin(), content.end());
    return payload;
  }

  void onSendTimer() {
    vision_interface::msg::Radar2Sentry radar_for_tx;
    bool using_fallback = false;
    if (!selectRadarForTx(radar_for_tx, using_fallback)) {
      return;
    }
    logTxSummary(radar_for_tx, using_fallback);
    logTxSlots(radar_for_tx, using_fallback);

    const int self_color = resolveSelfColor();
    if (self_color != 0 && self_color != 2) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "self_color unknown, skip serial TX. Set team/self_color_override correctly.");
      return;
    }

    if (!dry_run_ && !tryOpenSerial()) {
      return;
    }

    const auto payload = buildRadarPayload(radar_for_tx);
    const auto packet = buildPacket(payload, tx_cmd_id_);
    if (dry_run_) {
      publishTxRaw(packet);
    } else {
      const ssize_t n = ::write(fd_, packet.data(), packet.size());
      if (n < 0 || static_cast<size_t>(n) != packet.size()) {
        closeSerialWithWarn("serial write failed");
        return;
      }
      publishTxRaw(packet);
    }

    sendDecisionIfNeeded();
  }

  void sendDecisionIfNeeded() {
    if (!enable_decision_ || !has_pending_decision_) {
      return;
    }

    const uint16_t sender_id = resolveRadarRobotId();
    if (sender_id == 0U) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "self_color unknown, skip decision packet.");
      return;
    }

    const double now = this->get_clock()->now().seconds();
    if (now - last_decision_send_sec_ < decision_min_interval_s_) {
      return;
    }

    // 0x0121 雷达自主决策指令 (RM2026 通信协议 V1.3.0 §1.2 表 1-34):
    //   byte 0     : 双倍易伤触发计数 (单调递增 0→1→2...)
    //   byte 1     : 密钥指令类型 (0=空, 1=更新己方密钥, 2=验证破解的对方密钥)
    //   bytes 2-7  : 密钥 6 字节 (ASCII)
    // 【重要】byte 1-3 不是 dart/fly/hero 警告! 协议层规定为密钥, 全 0 表示空操作。
    // 总长 8 字节: radar_cmd(1) + password_cmd(1) + password_1..6(6)
    std::vector<uint8_t> content(8, 0);
    content[0] = pending_decision_byte_;
    // content[1..7] 保留全 0 (radar_cmd_t.password_cmd / password_1..6): 不参与密钥协议
    const auto interaction_payload = buildRobotInteractionPayload(
        decision_content_id_, sender_id, decision_receiver_id_, content);
    const auto pkt = buildPacket(interaction_payload, 0x0301U);

    if (dry_run_) {
      publishTxRaw(pkt);
    } else {
      const ssize_t n = ::write(fd_, pkt.data(), pkt.size());
      if (n < 0 || static_cast<size_t>(n) != pkt.size()) {
        closeSerialWithWarn("serial write failed for decision packet");
        return;
      }
    }

    last_decision_send_sec_ = now;
    has_pending_decision_ = false;
    RCLCPP_INFO(this->get_logger(), "TX decision packet sent. content_id=0x%04X value=%u",
                decision_content_id_, pending_decision_byte_);
  }

  void publishTxRaw(const std::vector<uint8_t> &pkt) {
    std_msgs::msg::UInt8MultiArray msg;
    msg.data = pkt;
    tx_raw_pub_->publish(msg);
  }

  void publishRxRaw(const uint8_t *data, size_t len) {
    if (!rx_raw_pub_ || data == nullptr || len == 0) {
      return;
    }
    std_msgs::msg::UInt8MultiArray msg;
    msg.data.assign(data, data + len);
    rx_raw_pub_->publish(msg);
  }

  void onRecvTimer() {
    if (!enable_receive_) {
      return;
    }

    if (!tryOpenSerial()) {
      return;
    }

    std::array<uint8_t, 512> buf {};
    size_t bytes_read_this_spin = 0;
    std::string first_chunk_hex;
    while (true) {
      const ssize_t n = ::read(fd_, buf.data(), buf.size());
      if (n > 0) {
        publishRxRaw(buf.data(), static_cast<size_t>(n));
        bytes_read_this_spin += static_cast<size_t>(n);
        if (first_chunk_hex.empty()) {
          first_chunk_hex = bytesToHex(buf.data(), std::min<size_t>(static_cast<size_t>(n), 24));
        }
        rx_buffer_.insert(rx_buffer_.end(), buf.begin(), buf.begin() + n);
      } else if (n == 0) {
        break;
      } else {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          break;
        }
        closeSerialWithWarn("serial read failed");
        return;
      }
    }

    if (bytes_read_this_spin > 0) {
      has_any_serial_rx_bytes_ = true;
      last_serial_rx_byte_sec_ = this->get_clock()->now().seconds();
      total_serial_rx_bytes_ += bytes_read_this_spin;
    }

    const bool had_valid_referee_before = has_any_referee_rx_;
    parseRxBuffer();
    if (bytes_read_this_spin > 0 && !has_any_referee_rx_ && !had_valid_referee_before) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "RX serial bytes seen on %s but no valid referee frame parsed yet. bytes=%zu first_chunk=%s",
          port_.c_str(), bytes_read_this_spin, first_chunk_hex.c_str());
    }
    reportRefereeRxDiagnostics();
    publishMatchInfoIfNeeded();
  }

  void reportRefereeRxDiagnostics() {
    const double now = this->get_clock()->now().seconds();
    if (now - last_referee_diag_sec_ < 2.0) {
      return;
    }
    last_referee_diag_sec_ = now;

    if (dry_run_ || !enable_receive_) {
      RCLCPP_WARN(this->get_logger(),
                  "【赛场体检-裁判系统】未真实接收：dry_run=%s enable_receive=%s。",
                  dry_run_ ? "true" : "false", enable_receive_ ? "true" : "false");
      return;
    }

    if (fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(),
                   "【赛场体检-裁判系统】串口未打开：port=%s。请检查 /dev/gimbal、USB转串口、udev。",
                   port_.c_str());
      return;
    }

    if (!has_any_serial_rx_bytes_) {
      RCLCPP_ERROR(this->get_logger(),
                   "【赛场体检-裁判系统】收不到任何字节：串口已打开 port=%s baud=%d，但裁判系统->电脑RX方向无数据。优先查 裁判TX->USB-RX、GND共地、线松、接错电管/主控接口。",
                   port_.c_str(), baud_rate_);
      return;
    }

    if (!has_any_referee_rx_) {
      RCLCPP_ERROR(this->get_logger(),
                   "【赛场体检-裁判系统】收到字节但不是合法裁判帧：total_rx=%zu 丢弃非A5字节=%zu CRC8错=%zu CRC16错=%zu 等待半帧=%zu。结论：有信号但协议/波特率/接线不对，常见为乱码、波特率不一致、接到非裁判常规链路。",
                   total_serial_rx_bytes_, rx_drop_no_sof_bytes_, rx_crc8_fail_count_,
                   rx_crc16_fail_count_, rx_incomplete_frame_count_);
      return;
    }

    const double age = now - last_referee_rx_sec_;
    if (age > 1.0) {
      RCLCPP_WARN(this->get_logger(),
                  "【赛场体检-裁判系统】之前解析成功，但最近 %.2fs 没有新合法帧。可能裁判系统下行间断、线松、链路刚断。",
                  age);
      return;
    }

    RCLCPP_INFO(this->get_logger(),
                "【赛场体检-裁判系统】正常：已收到并解析合法裁判帧，valid=%zu last_cmd=0x%04X last_len=%zu age=%.2fs total_rx=%zu。",
                rx_valid_frame_count_, last_referee_cmd_id_, last_referee_data_len_, age,
                total_serial_rx_bytes_);
  }

  void publishMatchInfoIfNeeded() {
    if (!publish_match_info_ || !match_info_pub_) {
      return;
    }

    const double now = this->get_clock()->now().seconds();
    if (has_any_referee_rx_ && (now - last_referee_rx_sec_ > match_info_timeout_s_)) {
      match_info_cache_.match_time = -200;
    }

    if (!dry_run_ && enable_receive_ && !has_any_referee_rx_ && (now - node_start_sec_ > 5.0)) {
      if (!has_any_serial_rx_bytes_) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "No serial RX bytes seen on %s since startup. TX packets may not be reaching a referee link.",
            port_.c_str());
      } else {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Serial RX bytes have been seen on %s (total=%zu) but no valid referee frame has been parsed.",
            port_.c_str(), total_serial_rx_bytes_);
      }
    }

    if (self_color_override_ == 0 || self_color_override_ == 2) {
      match_info_cache_.self_color = static_cast<int8_t>(self_color_override_);
    } else {
      match_info_cache_.self_color = static_cast<int8_t>(resolveSelfColor());
    }

    match_info_pub_->publish(match_info_cache_);
  }

  static uint16_t readU16LE(const uint8_t *p) {
    return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8U);
  }

  static uint32_t readU32LE(const uint8_t *p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8U) |
           (static_cast<uint32_t>(p[2]) << 16U) | (static_cast<uint32_t>(p[3]) << 24U);
  }

  void updateMatchInfoFromFrame(uint16_t cmd_id, const uint8_t *payload, size_t data_len) {
    if (!has_any_referee_rx_) {
      RCLCPP_INFO(this->get_logger(), "Referee RX detected. first_cmd=0x%04X len=%zu", cmd_id,
                  data_len);
    }
    has_any_referee_rx_ = true;
    last_referee_rx_sec_ = this->get_clock()->now().seconds();
    last_referee_cmd_id_ = cmd_id;
    last_referee_data_len_ = data_len;
    ++rx_valid_frame_count_;

    // 0x0001: game_status
    if (cmd_id == 0x0001U && data_len >= 3) {
      const uint8_t game_progress = static_cast<uint8_t>((payload[0] >> 4U) & 0x0FU);
      const uint16_t stage_remain_time = readU16LE(payload + 1);
      match_info_cache_.game_progress = game_progress;
      if (game_progress == 4U) {
        match_info_cache_.match_time = static_cast<int16_t>(stage_remain_time);
      } else if (game_progress == 5U) {
        match_info_cache_.match_time = -100;
      } else {
        match_info_cache_.match_time = -static_cast<int16_t>(stage_remain_time);
      }
      return;
    }

    // 0x0003: 己方 HP — 16 bytes (8 × u16), V1.3.0 表 1-8:
    //   [0]=ally hero1, [1]=ally eng2, [2]=ally inf3, [3]=ally inf4,
    //   [4]=reserved,   [5]=ally sentry7, [6]=ally outpost, [7]=ally base
    //   ⚠ 协议不提供敌方 HP, robot_hp[8..15] 保留不用 (zero-fill)。
    if (cmd_id == 0x0003U && data_len >= 16) {
      const size_t n = std::min<size_t>(data_len / 2, 8);
      for (size_t i = 0; i < n; ++i) {
        const uint16_t hp = readU16LE(payload + i * 2);
        match_info_cache_.robot_hp[i] = hp;
      }
      // [8..15] 协议无此数据, 始终清零, 下游须按 "己方 only" 处理。
      for (size_t i = 8; i < 16; ++i) {
        match_info_cache_.robot_hp[i] = 0;
      }
      return;
    }

    // 0x0101: event_data
    if (cmd_id == 0x0101U && data_len >= 4) {
      match_info_cache_.eventtype = readU32LE(payload);
      return;
    }

    // 0x0105: dart_info
    if (cmd_id == 0x0105U && data_len >= 3) {
      match_info_cache_.dart_remaining_time = payload[0];
      const uint16_t dart_info = readU16LE(payload + 1);
      match_info_cache_.dart_last_target = static_cast<uint8_t>(dart_info & 0x07U);
      match_info_cache_.dart_hit_count = static_cast<uint8_t>((dart_info >> 3U) & 0x07U);
      match_info_cache_.dart_selected_target =
          static_cast<uint8_t>((dart_info >> 6U) & 0x07U);
      return;
    }

    // 0x020C: mark bits (skip bit 4 = aerial)
    if (cmd_id == 0x020CU && data_len >= 2) {
      const uint16_t bits = readU16LE(payload);
      for (int pos = 0; pos < kProtoSlotCount; ++pos) {
        const int slot = kProtoPosToSlot[pos];
        if (slot < 0) continue;
        match_info_cache_.marks[slot] = static_cast<uint8_t>((bits >> pos) & 0x1U);
      }
      return;
    }

    // 0x020E: radar_info
    if (cmd_id == 0x020EU && data_len >= 1) {
      const uint8_t radar_info = payload[0];
      // bit 0-1: opportunity count (0~2), bit 2: double vuln active
      match_info_cache_.ultimate = static_cast<uint8_t>(radar_info & 0x07U);
      return;
    }
  }

  void parseRxBuffer() {
    constexpr size_t kHeaderLen = 5;
    constexpr size_t kCmdLen = 2;
    constexpr size_t kTailLen = 2;

    // 极端线路噪声/插拔时, rx_buffer_ 可能持续累积无效字节;
    // 直接 clip 到 8KB 上限, 避免内存增长。
    constexpr size_t kRxBufferHardCap = 8192;
    if (rx_buffer_.size() > kRxBufferHardCap) {
      rx_buffer_.erase(rx_buffer_.begin(),
                       rx_buffer_.end() - static_cast<ssize_t>(kRxBufferHardCap / 2));
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "rx_buffer exceeded %zu bytes, trimmed; possible serial noise.",
                           kRxBufferHardCap);
    }

    while (true) {
      if (rx_buffer_.size() < kHeaderLen + kCmdLen + kTailLen) {
        return;
      }

      auto sof_it = std::find(rx_buffer_.begin(), rx_buffer_.end(), 0xA5);
      if (sof_it == rx_buffer_.end()) {
        rx_drop_no_sof_bytes_ += rx_buffer_.size();
        rx_buffer_.clear();
        return;
      }

      if (sof_it != rx_buffer_.begin()) {
        rx_drop_no_sof_bytes_ += static_cast<size_t>(sof_it - rx_buffer_.begin());
        rx_buffer_.erase(rx_buffer_.begin(), sof_it);
      }

      if (rx_buffer_.size() < kHeaderLen + kCmdLen + kTailLen) {
        return;
      }

      const uint8_t expected_crc8 = crc8(rx_buffer_.data(), 4);
      if (expected_crc8 != rx_buffer_[4]) {
        ++rx_crc8_fail_count_;
        rx_buffer_.erase(rx_buffer_.begin());
        continue;
      }

      const uint16_t data_len =
          static_cast<uint16_t>(rx_buffer_[1]) | (static_cast<uint16_t>(rx_buffer_[2]) << 8U);
      const size_t frame_len = kHeaderLen + kCmdLen + data_len + kTailLen;
      if (rx_buffer_.size() < frame_len) {
        ++rx_incomplete_frame_count_;
        return;
      }

      const uint16_t frame_crc = static_cast<uint16_t>(rx_buffer_[frame_len - 2]) |
                                 (static_cast<uint16_t>(rx_buffer_[frame_len - 1]) << 8U);
      const uint16_t calc_crc = crc16(rx_buffer_.data(), frame_len - 2);
      if (frame_crc != calc_crc) {
        ++rx_crc16_fail_count_;
        rx_buffer_.erase(rx_buffer_.begin());
        continue;
      }

      const uint16_t cmd_id = static_cast<uint16_t>(rx_buffer_[5]) |
                              (static_cast<uint16_t>(rx_buffer_[6]) << 8U);
      const uint8_t *payload = rx_buffer_.data() + kHeaderLen + kCmdLen;

      updateMatchInfoFromFrame(cmd_id, payload, data_len);

      if (cmd_id == rx_expect_cmd_id_) {
        if (cmd_id == 0x0303U) {
          std_msgs::msg::UInt8MultiArray rx_msg;
          rx_msg.data.assign(payload, payload + data_len);
          rx_0303_pub_->publish(rx_msg);
        }

        if (log_rx_payload_hex_) {
          const std::string hex = bytesToHex(payload, data_len);
          RCLCPP_INFO(this->get_logger(), "RX cmd=0x%04X len=%u payload=%s", cmd_id, data_len,
                      hex.c_str());
        } else {
          RCLCPP_DEBUG(this->get_logger(), "RX cmd=0x%04X len=%u", cmd_id, data_len);
        }
      }

      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + frame_len);
    }
  }

private:
  std::string port_;
  int baud_rate_ = 115200;
  double send_hz_ = 5.0;
  double receive_hz_ = 50.0;
  double coord_scale_ = 100.0;
  double map_width_ = 28.0;
  double map_height_ = 15.0;
  uint16_t tx_cmd_id_ = 0x0305;
  std::string radar_topic_;
  std::string radar_topic_compat_;
  std::string send_match_topic_;
  int self_color_override_ = -1;

  bool enable_decision_ = true;
  std::string decision_topic_;
  uint16_t decision_receiver_id_ = 0x8080;
  uint16_t decision_content_id_ = 0x0121;
  double decision_min_interval_s_ = 0.10;
  bool publish_match_info_ = true;
  double match_info_timeout_s_ = 2.0;

  uint16_t rx_expect_cmd_id_ = 0x0303;
  bool enable_receive_ = true;
  bool log_rx_payload_hex_ = false;
  bool dry_run_ = false;

  int fd_ = -1;
  uint8_t seq_ = 0;
  double node_start_sec_ = 0.0;
  bool has_radar_direct_ = false;
  bool has_radar_fallback_ = false;
  bool has_nonzero_radar_fallback_ = false;
  bool has_match_info_ = false;
  vision_interface::msg::Radar2Sentry latest_radar_direct_;
  vision_interface::msg::Radar2Sentry latest_radar_fallback_;
  vision_interface::msg::Radar2Sentry latest_nonzero_radar_fallback_;
  std::array<double, kRobotSlotCount> fallback_enemy_slot_sec_ {};
  std::array<double, kRobotSlotCount> fallback_self_slot_sec_ {};
  vision_interface::msg::MatchInfo match_info_;
  bool enable_resolve_fallback_ = false;
  std::string resolve_topic_;
  std::string kalman_topic_;
  std::string map_points_topic_;
  double radar_topic_timeout_s_ = 0.5;
  double fallback_slot_hold_s_ = 30.0;
  double last_radar_direct_sec_ = -1e9;
  double last_radar_fallback_sec_ = -1e9;
  double last_nonzero_radar_fallback_sec_ = -1e9;

  bool has_pending_decision_ = false;
  uint8_t pending_decision_byte_ = 0;
  double last_decision_send_sec_ = -1e9;
  double last_tx_slots_log_sec_ = -1e9;
  bool has_any_serial_rx_bytes_ = false;
  double last_serial_rx_byte_sec_ = -1e9;
  size_t total_serial_rx_bytes_ = 0;
  bool has_any_referee_rx_ = false;
  double last_referee_rx_sec_ = -1e9;
  vision_interface::msg::MatchInfo match_info_cache_;
  double last_referee_diag_sec_ = -1e9;
  size_t rx_valid_frame_count_ = 0;
  size_t rx_crc8_fail_count_ = 0;
  size_t rx_crc16_fail_count_ = 0;
  size_t rx_drop_no_sof_bytes_ = 0;
  size_t rx_incomplete_frame_count_ = 0;
  uint16_t last_referee_cmd_id_ = 0;
  size_t last_referee_data_len_ = 0;

  std::vector<uint8_t> rx_buffer_;

  rclcpp::Subscription<vision_interface::msg::MatchInfo>::SharedPtr match_sub_;
  rclcpp::Subscription<vision_interface::msg::Radar2Sentry>::SharedPtr radar_sub_;
  rclcpp::Subscription<vision_interface::msg::Radar2Sentry>::SharedPtr radar_sub_compat_;
  rclcpp::Subscription<vision_interface::msg::DetectResult>::SharedPtr map_points_sub_;
  rclcpp::Subscription<vision_interface::msg::DetectResult>::SharedPtr kalman_sub_;
  rclcpp::Subscription<vision_interface::msg::DetectResult>::SharedPtr resolve_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr decision_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr rx_0303_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr rx_raw_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr tx_raw_pub_;
  rclcpp::Publisher<vision_interface::msg::MatchInfo>::SharedPtr match_info_pub_;
  rclcpp::TimerBase::SharedPtr send_timer_;
  rclcpp::TimerBase::SharedPtr recv_timer_;
};

}  // namespace tdt_radar

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tdt_radar::RadarSerialNode>());
  rclcpp::shutdown();
  return 0;
}
