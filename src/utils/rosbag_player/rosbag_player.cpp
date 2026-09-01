#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <cv_bridge/cv_bridge.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vision_interface/msg/match_info.hpp>

using namespace std::chrono_literals;

namespace {

builtin_interfaces::msg::Time to_stamp(uint64_t timestamp_ns)
{
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(timestamp_ns / 1000000000ULL);
    stamp.nanosec = static_cast<uint32_t>(timestamp_ns % 1000000000ULL);
    return stamp;
}

}  // namespace

class RosbagPlayer : public rclcpp::Node {
public:
    RosbagPlayer(const rclcpp::NodeOptions& options)
        : Node("rosbag_player_node", options)
    {
        this->declare_parameter<std::string>("rosbag_file", "");
        this->declare_parameter<bool>("loop_playback", true);
        this->declare_parameter<bool>("use_bag_timing", true);
        this->declare_parameter<double>("replay_rate", 1.0);
        this->declare_parameter<int>("max_sleep_ms", 300);
        this->declare_parameter<bool>("decode_compressed_image", true);
        this->declare_parameter<bool>("publish_compressed_image", true);
        this->declare_parameter<double>("camera_max_fps", 20.0);
        this->declare_parameter<bool>("publish_lidar", true);
        this->declare_parameter<bool>("publish_match_info", true);
        this->declare_parameter<bool>("wait_on_lidar_timing", true);
        this->declare_parameter<bool>("legacy_mode", true);
        this->declare_parameter<int>("legacy_cycle_ms", 100);
        this->declare_parameter<double>("start_offset_sec", 0.0);
        this->declare_parameter<std::vector<std::string>>(
            "passthrough_topics",
            std::vector<std::string>{});

        this->get_parameter("rosbag_file", rosbag_file);
        this->get_parameter("loop_playback", loop_playback_);
        this->get_parameter("use_bag_timing", use_bag_timing_);
        this->get_parameter("replay_rate", replay_rate_);
        this->get_parameter("max_sleep_ms", max_sleep_ms_);
        this->get_parameter("decode_compressed_image", decode_compressed_image_);
        this->get_parameter("publish_compressed_image", publish_compressed_image_);
        this->get_parameter("camera_max_fps", camera_max_fps_);
        this->get_parameter("publish_lidar", publish_lidar_);
        this->get_parameter("publish_match_info", publish_match_info_);
        this->get_parameter("wait_on_lidar_timing", wait_on_lidar_timing_);
        this->get_parameter("legacy_mode", legacy_mode_);
        this->get_parameter("legacy_cycle_ms", legacy_cycle_ms_);
        this->get_parameter("start_offset_sec", start_offset_sec_);
        this->get_parameter("passthrough_topics", passthrough_topics_);

        if (replay_rate_ <= 0.0) {
            replay_rate_ = 1.0;
            RCLCPP_WARN(
                this->get_logger(),
                "Invalid replay_rate<=0, fallback to 1.0");
        }

        pointcloud_publisher_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/livox/lidar", 10);
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            "camera_image", rclcpp::SensorDataQoS());
        compressed_image_publisher_ =
            this->create_publisher<sensor_msgs::msg::CompressedImage>(
                "compressed_image", rclcpp::SensorDataQoS());
        match_info_publisher_ =
            this->create_publisher<vision_interface::msg::MatchInfo>(
                "/match_info", 10);

        open_reader();
        init_topic_type_map_and_passthrough_publishers();

        RCLCPP_INFO(
            this->get_logger(),
            "rosbag=%s, use_bag_timing=%d, replay_rate=%.2f, camera_max_fps=%.1f, decode_compressed_image=%d, publish_lidar=%d, wait_on_lidar_timing=%d, legacy_mode=%d, legacy_cycle_ms=%d, start_offset_sec=%.1f, passthrough_topics=%zu",
            rosbag_file.c_str(),
            use_bag_timing_,
            replay_rate_,
            camera_max_fps_,
            decode_compressed_image_,
            publish_lidar_ ? 1 : 0,
            wait_on_lidar_timing_ ? 1 : 0,
            legacy_mode_ ? 1 : 0,
            legacy_cycle_ms_,
            start_offset_sec_,
            passthrough_topics_.size());
        RCLCPP_INFO(
            this->get_logger(),
            "rosbag_player extra: publish_match_info=%d",
            publish_match_info_ ? 1 : 0);

        decode_thread_ =
            std::make_shared<std::thread>(&RosbagPlayer::decode_loop, this);
        processing_thread_ =
            std::make_shared<std::thread>(&RosbagPlayer::play_bag, this);
    }

    ~RosbagPlayer()
    {
        stop_requested_.store(true);
        {
            std::lock_guard<std::mutex> lk(decode_queue_mu_);
            decode_queue_cv_.notify_all();
        }
        if (processing_thread_ && processing_thread_->joinable()) {
            processing_thread_->join();
        }
        if (decode_thread_ && decode_thread_->joinable()) {
            decode_thread_->join();
        }
    }

private:
    std::string normalize_topic(const std::string& topic) const
    {
        if (topic.empty() || topic[0] == '/') {
            return topic;
        }
        return "/" + topic;
    }

    bool is_core_topic(const std::string& topic) const
    {
        const auto n = normalize_topic(topic);
        return n == "/livox/lidar" || n == "/compressed_image" ||
               n == "/camera_image" || n == "/match_info";
    }

    bool is_camera_topic(const std::string& topic) const
    {
        const auto n = normalize_topic(topic);
        return n == "/camera_image" || n == "/compressed_image";
    }

    void open_reader()
    {
        reader_.open(rosbag_file);
        reset_bag_timing();
    }

    void reopen_reader_for_loop()
    {
        reader_.close();
        reader_.open(rosbag_file);
        reset_bag_timing();
    }

    void reset_bag_timing()
    {
        has_last_bag_ts_ = false;
        last_bag_timestamp_ns_ = 0;
        has_first_bag_ts_ = false;
        first_bag_timestamp_ns_ = 0;
        has_last_camera_ts_ = false;
        last_camera_ts_ns_ = 0;
    }

    void init_topic_type_map_and_passthrough_publishers()
    {
        topic_type_map_.clear();
        passthrough_publishers_.clear();
        passthrough_set_.clear();

        const auto all_topics = reader_.get_all_topics_and_types();
        for (const auto& topic_meta : all_topics) {
            topic_type_map_[normalize_topic(topic_meta.name)] = topic_meta.type;
        }

        for (const auto& topic_name_raw : passthrough_topics_) {
            const auto topic_name = normalize_topic(topic_name_raw);
            if (topic_name.empty() || is_core_topic(topic_name)) {
                continue;
            }
            passthrough_set_.insert(topic_name);

            const auto iter = topic_type_map_.find(topic_name);
            if (iter == topic_type_map_.end()) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Passthrough topic '%s' is not in bag metadata",
                    topic_name.c_str());
                continue;
            }

            passthrough_publishers_[topic_name] =
                this->create_generic_publisher(topic_name, iter->second, 10);
        }
    }

    uint64_t resolve_bag_timestamp_ns(
        const std::shared_ptr<rosbag2_storage::SerializedBagMessage>& bag_message)
    {
        if (bag_message->recv_timestamp > 0U) {
            return bag_message->recv_timestamp;
        }
        if (bag_message->send_timestamp > 0U) {
            return bag_message->send_timestamp;
        }
        return 0U;
    }

    bool should_publish_camera_message(
        const std::shared_ptr<rosbag2_storage::SerializedBagMessage>& bag_message)
    {
        if (camera_max_fps_ <= 0.0) {
            return true;
        }

        const uint64_t now_ts_ns = resolve_bag_timestamp_ns(bag_message);
        uint64_t current_ts_ns = now_ts_ns;
        if (current_ts_ns == 0U) {
            current_ts_ns = static_cast<uint64_t>(
                this->get_clock()->now().nanoseconds());
        }

        if (!has_last_camera_ts_) {
            has_last_camera_ts_ = true;
            last_camera_ts_ns_ = current_ts_ns;
            return true;
        }

        const uint64_t min_delta_ns = static_cast<uint64_t>(
            std::max(1.0, 1e9 / camera_max_fps_));
        if (current_ts_ns > last_camera_ts_ns_ &&
            (current_ts_ns - last_camera_ts_ns_) < min_delta_ns) {
            ++dropped_camera_msgs_;
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 3000,
                "Camera replay throttled: dropped=%zu max_fps=%.1f",
                dropped_camera_msgs_, camera_max_fps_);
            return false;
        }

        last_camera_ts_ns_ = current_ts_ns;
        return true;
    }

    void wait_by_bag_timing(
        const std::shared_ptr<rosbag2_storage::SerializedBagMessage>& bag_message)
    {
        if (!use_bag_timing_) {
            return;
        }

        const auto current_ts_ns = resolve_bag_timestamp_ns(bag_message);
        if (current_ts_ns == 0U) {
            return;
        }

        const auto now_wall_ns = static_cast<uint64_t>(
            std::chrono::steady_clock::now().time_since_epoch().count());

        if (!has_last_bag_ts_) {
            has_last_bag_ts_ = true;
            last_bag_timestamp_ns_ = current_ts_ns;
            start_bag_ns_ = current_ts_ns;
            start_wall_ns_ = now_wall_ns;
            return;
        }

        // If time jumps back significantly, just reset baseline
        if (current_ts_ns < last_bag_timestamp_ns_) {
            const int64_t diff = last_bag_timestamp_ns_ - current_ts_ns;
            if (diff > 1000000000LL) { // Jump back > 1s
                start_bag_ns_ = current_ts_ns;
                start_wall_ns_ = now_wall_ns;
            }
        }
        last_bag_timestamp_ns_ = current_ts_ns;

        if (current_ts_ns <= start_bag_ns_) {
            return;
        }

        const auto elapsed_bag_ns = current_ts_ns - start_bag_ns_;
        const auto expected_elapsed_wall_ns = static_cast<uint64_t>(
            static_cast<double>(elapsed_bag_ns) / replay_rate_);
        
        const auto expected_wall_ns = start_wall_ns_ + expected_elapsed_wall_ns;

        if (expected_wall_ns > now_wall_ns) {
            uint64_t sleep_ns = expected_wall_ns - now_wall_ns;
            
            const auto max_sleep_ns = static_cast<uint64_t>(
                std::max(0, max_sleep_ms_)) * 1000000ULL;
                
            if (max_sleep_ns > 0U && sleep_ns > max_sleep_ns) {
                sleep_ns = max_sleep_ns;
            }
            
            if (sleep_ns > 0U) {
                std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_ns));
            }
        }
    }

    template <typename T>
    std::shared_ptr<T> deserialize_message(
        const std::shared_ptr<rosbag2_storage::SerializedBagMessage>& bag_message)
    {
        auto msg = std::make_shared<T>();
        rclcpp::Serialization<T> serialization;
        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
        serialization.deserialize_message(&serialized_msg, msg.get());
        return msg;
    }

    void publish_passthrough_if_configured(
        const std::shared_ptr<rosbag2_storage::SerializedBagMessage>& bag_message,
        const std::string& topic_name)
    {
        const auto normalized = normalize_topic(topic_name);
        if (passthrough_set_.find(normalized) == passthrough_set_.end()) {
            return;
        }

        const auto pub_iter = passthrough_publishers_.find(normalized);
        if (pub_iter == passthrough_publishers_.end()) {
            return;
        }

        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
        pub_iter->second->publish(serialized_msg);
    }

    void decode_loop()
    {
        while (rclcpp::ok() && !stop_requested_.load()) {
            DecodeJob job;
            {
                std::unique_lock<std::mutex> lk(decode_queue_mu_);
                decode_queue_cv_.wait(lk, [this]() {
                    return stop_requested_.load() || !decode_queue_.empty();
                });
                if (stop_requested_.load() && decode_queue_.empty()) {
                    return;
                }
                job = std::move(decode_queue_.front());
                decode_queue_.pop_front();
            }

            cv::Mat img = cv::imdecode(job.data, cv::IMREAD_COLOR);
            if (img.empty()) continue;

            // Build Image message directly into unique_ptr (single copy of pixels).
            auto image_msg = std::make_unique<sensor_msgs::msg::Image>();
            image_msg->header.stamp = job.stamp;
            image_msg->header.frame_id = "camera";
            image_msg->height = static_cast<uint32_t>(img.rows);
            image_msg->width = static_cast<uint32_t>(img.cols);
            image_msg->encoding = "bgr8";
            image_msg->is_bigendian = false;
            image_msg->step =
                static_cast<uint32_t>(img.cols * img.channels());
            const size_t total =
                static_cast<size_t>(image_msg->step) * image_msg->height;
            image_msg->data.resize(total);
            if (img.isContinuous()) {
                std::memcpy(image_msg->data.data(), img.data, total);
            } else {
                size_t offset = 0;
                for (int r = 0; r < img.rows; ++r) {
                    std::memcpy(
                        image_msg->data.data() + offset,
                        img.ptr(r),
                        image_msg->step);
                    offset += image_msg->step;
                }
            }
            // Zero-copy publish via unique_ptr move (intra-process).
            image_publisher_->publish(std::move(image_msg));
        }
    }

    void play_bag()
    {
        while (rclcpp::ok() && !stop_requested_.load()) {
            if (!reader_.has_next()) {
                if (!loop_playback_) {
                    break;
                }
                reopen_reader_for_loop();
                continue;
            }

            auto cycle_start = std::chrono::steady_clock::now();
            auto bag_message = reader_.read_next();
            const auto topic_name = normalize_topic(bag_message->topic_name);
            const auto raw_bag_timestamp_ns = resolve_bag_timestamp_ns(bag_message);
            if (start_offset_sec_ > 0.0 && raw_bag_timestamp_ns > 0U) {
                if (!has_first_bag_ts_) {
                    has_first_bag_ts_ = true;
                    first_bag_timestamp_ns_ = raw_bag_timestamp_ns;
                }
                const uint64_t offset_ns =
                    static_cast<uint64_t>(start_offset_sec_ * 1000000000.0);
                if (raw_bag_timestamp_ns < first_bag_timestamp_ns_ + offset_ns) {
                    continue;
                }
            }
            if (!legacy_mode_) {
                const bool skip_lidar_timing =
                    (topic_name == "/livox/lidar" && !wait_on_lidar_timing_);
                if (!skip_lidar_timing) {
                    wait_by_bag_timing(bag_message);
                }
            }
            if (!rclcpp::ok() || stop_requested_.load()) {
                break;
            }
            const auto bag_timestamp_ns = raw_bag_timestamp_ns;
            const auto replay_stamp = bag_timestamp_ns > 0U
                ? to_stamp(bag_timestamp_ns)
                : to_stamp(static_cast<uint64_t>(this->get_clock()->now().nanoseconds()));

            if (topic_name == "/livox/lidar") {
                if (!publish_lidar_) {
                    continue;
                }
                const bool has_lidar_subscribers =
                    pointcloud_publisher_->get_subscription_count() > 0 ||
                    pointcloud_publisher_->get_intra_process_subscription_count() > 0;
                if (!has_lidar_subscribers) {
                    continue;
                }
                auto pointcloud_msg =
                    deserialize_message<sensor_msgs::msg::PointCloud2>(
                        bag_message);
                pointcloud_msg->header.stamp = replay_stamp;
                pointcloud_publisher_->publish(*pointcloud_msg);
            }
            else if (topic_name == "/camera_image") {
                if (!should_publish_camera_message(bag_message)) {
                    continue;
                }
                // Zero-copy publish via unique_ptr move (intra-process).
                auto image_msg = std::make_unique<sensor_msgs::msg::Image>();
                rclcpp::Serialization<sensor_msgs::msg::Image> ser;
                rclcpp::SerializedMessage smsg(*bag_message->serialized_data);
                ser.deserialize_message(&smsg, image_msg.get());
                image_msg->header.stamp = replay_stamp;
                image_publisher_->publish(std::move(image_msg));
            }
            else if (topic_name == "/compressed_image") {
                if (!should_publish_camera_message(bag_message)) {
                    continue;
                }
                auto compressed_msg =
                    deserialize_message<sensor_msgs::msg::CompressedImage>(
                        bag_message);

                if (publish_compressed_image_) {
                    compressed_msg->header.stamp = replay_stamp;
                    compressed_image_publisher_->publish(*compressed_msg);
                }

                if (decode_compressed_image_) {
                    // Hand off to decode thread to keep play_bag responsive.
                    DecodeJob job;
                    job.data = std::move(compressed_msg->data);
                    job.stamp = replay_stamp;
                    {
                        std::lock_guard<std::mutex> lk(decode_queue_mu_);
                        // Drop-oldest if backlogged to bound latency & memory.
                        while (decode_queue_.size() >= decode_queue_max_) {
                            decode_queue_.pop_front();
                            ++dropped_decode_jobs_;
                        }
                        decode_queue_.push_back(std::move(job));
                    }
                    decode_queue_cv_.notify_one();
                    if (dropped_decode_jobs_ > 0 &&
                        dropped_decode_jobs_ % 30 == 0) {
                        RCLCPP_WARN_THROTTLE(
                            this->get_logger(), *this->get_clock(), 3000,
                            "Decode backlog: dropped=%zu (queue cap=%zu)",
                            dropped_decode_jobs_, decode_queue_max_);
                    }
                }
            }
            else if (topic_name == "/match_info") {
                if (!publish_match_info_) {
                    continue;
                }
                try {
                    auto match_info_msg =
                        deserialize_message<vision_interface::msg::MatchInfo>(
                            bag_message);
                    match_info_publisher_->publish(*match_info_msg);
                } catch (const std::exception& e) {
                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(), *this->get_clock(), 3000,
                        "Skip /match_info from bag due to deserialize failure: %s",
                        e.what());
                }
            }
            else {
                publish_passthrough_if_configured(bag_message, topic_name);
            }

            if (legacy_mode_ && legacy_cycle_ms_ > 0 &&
                is_camera_topic(topic_name)) {
                const auto elapsed_us =
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now() - cycle_start)
                        .count();
                const long target_us = static_cast<long>(legacy_cycle_ms_) * 1000L;
                if (elapsed_us < target_us) {
                    std::this_thread::sleep_for(
                        std::chrono::microseconds(target_us - elapsed_us));
                }
            }
        }
        if (rclcpp::ok() && !stop_requested_.load()) {
            RCLCPP_INFO(this->get_logger(), "No more messages in the bag.");
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        pointcloud_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr
        compressed_image_publisher_;
    rclcpp::Publisher<vision_interface::msg::MatchInfo>::SharedPtr
                                 match_info_publisher_;
    std::unordered_map<std::string, std::string> topic_type_map_;
    std::unordered_set<std::string> passthrough_set_;
    std::unordered_map<std::string, rclcpp::GenericPublisher::SharedPtr>
        passthrough_publishers_;
    rosbag2_cpp::Reader          reader_;
    std::shared_ptr<std::thread> processing_thread_;
    std::shared_ptr<std::thread> decode_thread_;
    struct DecodeJob {
        std::vector<uint8_t>           data;
        builtin_interfaces::msg::Time  stamp;
    };
    std::deque<DecodeJob>        decode_queue_;
    std::mutex                   decode_queue_mu_;
    std::condition_variable      decode_queue_cv_;
    static constexpr size_t      decode_queue_max_ = 2;
    size_t                       dropped_decode_jobs_{0};
    std::string                  rosbag_file;
    std::vector<std::string>     passthrough_topics_;
    bool                         loop_playback_{true};
    bool                         use_bag_timing_{true};
    bool                         decode_compressed_image_{true};
    bool                         publish_compressed_image_{true};
    double                       camera_max_fps_{20.0};
    bool                         publish_lidar_{true};
    bool                         publish_match_info_{true};
    bool                         wait_on_lidar_timing_{true};
    bool                         legacy_mode_{true};
    int                          legacy_cycle_ms_{100};
    double                       start_offset_sec_{0.0};
    double                       replay_rate_{1.0};
    int                          max_sleep_ms_{300};
    std::atomic<bool>            stop_requested_{false};
    bool                         has_last_bag_ts_{false};
    uint64_t                     last_bag_timestamp_ns_{0U};
    uint64_t                     start_bag_ns_{0U};
    uint64_t                     start_wall_ns_{0U};
    bool                         has_first_bag_ts_{false};
    uint64_t                     first_bag_timestamp_ns_{0U};
    bool                         has_last_camera_ts_{false};
    uint64_t                     last_camera_ts_ns_{0U};
    size_t                       dropped_camera_msgs_{0U};
};

RCLCPP_COMPONENTS_REGISTER_NODE(RosbagPlayer)
