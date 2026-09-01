#include <sys/stat.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>

#include <chrono>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <system_error>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/topic_metadata.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <vision_interface/msg/detect_result.hpp>
#include <vision_interface/msg/match_info.hpp>
#include <vision_interface/msg/radar2_sentry.hpp>

namespace {

std::string now_string()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t t = std::chrono::system_clock::to_time_t(now);
  struct tm tm_buf;
  localtime_r(&t, &tm_buf);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm_buf);
  return std::string(buf);
}

void ensure_dir(const std::string &dir, const rclcpp::Logger &logger)
{
  // 废除 system("mkdir -p") 路径: 避免 shell 转义 + 丢失错误诊断。
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  if (ec) {
    RCLCPP_ERROR(logger, "Failed to create bag dir '%s': %s",
                 dir.c_str(), ec.message().c_str());
  }
}

void refresh_latest_link(const std::string &dir, const std::string &target,
                         const rclcpp::Logger &logger)
{
  const std::string latest = dir + "/latest";
  std::error_code ec;
  const auto latest_status = std::filesystem::symlink_status(latest, ec);
  if (!ec && std::filesystem::exists(latest_status)) {
    if (std::filesystem::is_symlink(latest_status)) {
      std::filesystem::remove(latest, ec);
      if (ec) {
        RCLCPP_WARN(logger, "remove symlink(%s) failed: %s",
                    latest.c_str(), ec.message().c_str());
      }
    } else {
      const std::string backup = latest + ".preserved";
      std::filesystem::rename(latest, backup, ec);
      if (ec) {
        RCLCPP_WARN(logger, "latest exists and is not symlink; preserve failed %s -> %s: %s",
                    latest.c_str(), backup.c_str(), ec.message().c_str());
        return;
      }
      RCLCPP_WARN(logger, "latest existed as real path; renamed to %s to avoid data loss",
                  backup.c_str());
    }
  }
  if (::symlink(target.c_str(), latest.c_str()) != 0) {
    RCLCPP_WARN(logger, "symlink(%s -> %s) failed: %s",
                latest.c_str(), target.c_str(), std::strerror(errno));
  }
}

}  // namespace

class BagRecorderNode : public rclcpp::Node {
public:
  explicit BagRecorderNode(const rclcpp::NodeOptions &options)
  : Node("record_node", options)
  {
    const std::string bag_dir =
      this->declare_parameter<std::string>("bag_dir", "/home/zst/T/bags");
    const std::string bag_prefix =
      this->declare_parameter<std::string>("bag_prefix", "match");
    const std::string storage_id =
      this->declare_parameter<std::string>("storage_id", "mcap");
    image_record_hz_ = this->declare_parameter<double>("image_record_hz", 10.0);
    if (image_record_hz_ < 0.0) {
      image_record_hz_ = 0.0;
    }

    ensure_dir(bag_dir, this->get_logger());
    bag_uri_ = bag_dir + "/" + bag_prefix + "_" + now_string();
    refresh_latest_link(bag_dir, bag_uri_, this->get_logger());

    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_uri_;
    storage_options.storage_id = storage_id;

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open(storage_options, converter_options);

    create_topic("/livox/lidar", "sensor_msgs/msg/PointCloud2");
    create_topic("/compressed_image", "sensor_msgs/msg/CompressedImage");
    create_topic("/match_info", "vision_interface/msg/MatchInfo");
    create_topic("/resolve_result", "vision_interface/msg/DetectResult");
    create_topic("/kalman_detect", "vision_interface/msg/DetectResult");
    create_topic("/radar2sentry", "vision_interface/msg/Radar2Sentry");
    create_topic("/radar/rx_raw", "std_msgs/msg/UInt8MultiArray");
    create_topic("/radar/tx_raw", "std_msgs/msg/UInt8MultiArray");

    sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar", rclcpp::SensorDataQoS(),
      std::bind(&BagRecorderNode::on_lidar, this, std::placeholders::_1));

    sub_image_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/compressed_image", rclcpp::SensorDataQoS(),
      std::bind(&BagRecorderNode::on_image, this, std::placeholders::_1));

    sub_match_ = this->create_subscription<vision_interface::msg::MatchInfo>(
      "/match_info", rclcpp::SensorDataQoS(),
      std::bind(&BagRecorderNode::on_match, this, std::placeholders::_1));

    sub_resolve_ = this->create_subscription<vision_interface::msg::DetectResult>(
      "/resolve_result", rclcpp::SensorDataQoS(),
      std::bind(&BagRecorderNode::on_resolve, this, std::placeholders::_1));

    sub_kalman_ = this->create_subscription<vision_interface::msg::DetectResult>(
      "/kalman_detect", rclcpp::SensorDataQoS(),
      std::bind(&BagRecorderNode::on_kalman, this, std::placeholders::_1));

    sub_radar_ = this->create_subscription<vision_interface::msg::Radar2Sentry>(
      "/radar2sentry", rclcpp::SensorDataQoS(),
      std::bind(&BagRecorderNode::on_radar, this, std::placeholders::_1));

    sub_rx_raw_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
      "/radar/rx_raw", 10,
      std::bind(&BagRecorderNode::on_rx_raw, this, std::placeholders::_1));

    sub_tx_raw_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
      "/radar/tx_raw", 10,
      std::bind(&BagRecorderNode::on_tx_raw, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "BagRecorderNode started, uri=%s image_record_hz=%.1f",
                bag_uri_.c_str(), image_record_hz_);
  }

private:
  void create_topic(const std::string &name, const std::string &type)
  {
    rosbag2_storage::TopicMetadata meta;
    meta.name = name;
    meta.type = type;
    meta.serialization_format = "cdr";
    writer_->create_topic(meta);
  }

  // 使用 msg.header.stamp 作为 send_timestamp (传感器采集时闻), 让 use_bag_timing
  // 回放严格复现原始时序; recv_timestamp 用 now() 反映节点接收时间。
  // 不带 header 的消息 (MatchInfo / Radar2Sentry / UInt8MultiArray) 两者同为 now()。
  template<typename MsgT>
  void write_msg(const std::string &topic, const MsgT &msg, int64_t send_ns = 0)
  {
    rclcpp::SerializedMessage serialized;
    rclcpp::Serialization<MsgT> serializer;
    serializer.serialize_message(&msg, &serialized);

    auto bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    const auto now_ns = this->get_clock()->now().nanoseconds();
    bag_msg->recv_timestamp = now_ns;
    bag_msg->send_timestamp = (send_ns > 0) ? send_ns : now_ns;
    bag_msg->topic_name = topic;
    bag_msg->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
      new rcutils_uint8_array_t,
      [](rcutils_uint8_array_t * data) {
        if (data != nullptr) {
          const auto ret = rcutils_uint8_array_fini(data);
          (void)ret;
          delete data;
        }
      });
    *bag_msg->serialized_data = serialized.release_rcl_serialized_message();

    std::lock_guard<std::mutex> lk(write_mutex_);
    writer_->write(bag_msg);
  }

  static int64_t header_ns(const std_msgs::msg::Header &h)
  {
    return static_cast<int64_t>(h.stamp.sec) * 1000000000LL +
           static_cast<int64_t>(h.stamp.nanosec);
  }

  void on_lidar(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    write_msg("/livox/lidar", *msg, header_ns(msg->header));
  }

  void on_image(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    const int64_t stamp_ns = header_ns(msg->header);
    const int64_t now_ns = this->get_clock()->now().nanoseconds();
    const int64_t t_ns = (stamp_ns > 0) ? stamp_ns : now_ns;
    if (image_record_hz_ > 0.0) {
      const int64_t min_interval_ns =
        static_cast<int64_t>(1000000000.0 / image_record_hz_);
      if (last_image_record_ns_ > 0 && (t_ns - last_image_record_ns_) < min_interval_ns) {
        return;
      }
      last_image_record_ns_ = t_ns;
    }
    write_msg("/compressed_image", *msg, header_ns(msg->header));
  }

  void on_match(const vision_interface::msg::MatchInfo::SharedPtr msg)
  {
    write_msg("/match_info", *msg);  // no header
  }

  void on_resolve(const vision_interface::msg::DetectResult::SharedPtr msg)
  {
    write_msg("/resolve_result", *msg, header_ns(msg->header));
  }

  void on_kalman(const vision_interface::msg::DetectResult::SharedPtr msg)
  {
    write_msg("/kalman_detect", *msg, header_ns(msg->header));
  }

  void on_radar(const vision_interface::msg::Radar2Sentry::SharedPtr msg)
  {
    write_msg("/radar2sentry", *msg);  // no header
  }

  void on_rx_raw(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
  {
    write_msg("/radar/rx_raw", *msg);  // no header
  }

  void on_tx_raw(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
  {
    write_msg("/radar/tx_raw", *msg);  // no header
  }

private:
  std::string bag_uri_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  std::mutex write_mutex_;
  double image_record_hz_ = 10.0;
  int64_t last_image_record_ns_ = 0;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_image_;
  rclcpp::Subscription<vision_interface::msg::MatchInfo>::SharedPtr sub_match_;
  rclcpp::Subscription<vision_interface::msg::DetectResult>::SharedPtr sub_resolve_;
  rclcpp::Subscription<vision_interface::msg::DetectResult>::SharedPtr sub_kalman_;
  rclcpp::Subscription<vision_interface::msg::Radar2Sentry>::SharedPtr sub_radar_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_rx_raw_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_tx_raw_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(BagRecorderNode)
