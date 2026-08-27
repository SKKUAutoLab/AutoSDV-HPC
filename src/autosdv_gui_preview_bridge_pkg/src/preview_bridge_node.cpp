#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

class PreviewBridgeNode : public rclcpp::Node
{
public:
  PreviewBridgeNode()
  : Node("autosdv_gui_preview_bridge")
  {
    jpeg_quality_ = declare_parameter<int>("jpeg_quality", 85);
    jpeg_quality_ = std::max(1, std::min(100, jpeg_quality_));

    const auto image_qos = rclcpp::SensorDataQoS().keep_last(1);
    output_publisher_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      "/autosdv_gui/preview/compressed", image_qos);

    const auto source_qos = rclcpp::QoS(1).reliable().transient_local();
    source_subscription_ = create_subscription<std_msgs::msg::String>(
      "/autosdv_gui/preview/source",
      source_qos,
      std::bind(&PreviewBridgeNode::select_source, this, std::placeholders::_1));
  }

private:
  void select_source(const std_msgs::msg::String::ConstSharedPtr message)
  {
    if (message->data == input_topic_) {
      return;
    }

    image_subscription_.reset();
    input_topic_ = message->data;
    if (input_topic_.empty()) {
      RCLCPP_INFO(get_logger(), "Preview source cleared");
      return;
    }

    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      input_topic_,
      rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&PreviewBridgeNode::image_callback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Preview source: %s", input_topic_.c_str());
  }

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr message)
  {
    try {
      const auto image = cv_bridge::toCvShare(
        message, sensor_msgs::image_encodings::BGR8);
      sensor_msgs::msg::CompressedImage output;
      output.header = message->header;
      output.format = "jpeg";
      const std::vector<int> parameters = {
        cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
      if (!cv::imencode(".jpg", image->image, output.data, parameters)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "JPEG preview encoding failed");
        return;
      }
      output_publisher_->publish(std::move(output));
    } catch (const cv_bridge::Exception & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Preview conversion failed: %s", exception.what());
    } catch (const cv::Exception & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Preview JPEG encoding failed: %s", exception.what());
    }
  }

  int jpeg_quality_;
  std::string input_topic_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr output_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr source_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreviewBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
