#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace tdt_radar {

class DetectDebugView final : public rclcpp::Node {
public:
    explicit DetectDebugView(const rclcpp::NodeOptions& options)
        : Node("detect_debug_view", options)
    {
        topic_ = this->declare_parameter<std::string>("image_topic", "detect_debug_image");
        window_name_ = this->declare_parameter<std::string>("window_name", "detect");
        cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            topic_, rclcpp::SensorDataQoS(),
            std::bind(&DetectDebugView::callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "DetectDebugView showing %s", topic_.c_str());
    }

    ~DetectDebugView() override
    {
        cv::destroyWindow(window_name_);
    }

private:
    void callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            auto image = cv_bridge::toCvShare(msg, "bgr8")->image;
            if (!image.empty()) {
                cv::imshow(window_name_, image);
                cv::waitKey(1);
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "debug image display failed: %s", e.what());
        }
    }

    std::string topic_;
    std::string window_name_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

}  // namespace tdt_radar

RCLCPP_COMPONENTS_REGISTER_NODE(tdt_radar::DetectDebugView)
