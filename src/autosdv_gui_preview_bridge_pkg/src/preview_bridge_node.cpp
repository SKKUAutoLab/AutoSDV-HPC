#include <algorithm>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

namespace
{

// Convert common sensor_msgs/Image encodings without cv_bridge. The ROS
// libcv_bridge.so is built against OpenCV 4.6 while this workspace compiles
// against the 4.8 headers shipped by JetPack, so linking it would pull both
// copies of libopencv_core into one process. This mirrors the encoding set
// that ros_image_to_qimage() in autosdv_gui_pkg already handles.
cv::Mat to_bgr8(const sensor_msgs::msg::Image & message)
{
  std::string encoding = message.encoding;
  std::transform(
    encoding.begin(), encoding.end(), encoding.begin(),
    [](unsigned char character) {return static_cast<char>(std::tolower(character));});

  int source_type = 0;
  int conversion = -1;
  if (encoding == "bgr8" || encoding == "8uc3") {
    source_type = CV_8UC3;
  } else if (encoding == "rgb8") {
    source_type = CV_8UC3;
    conversion = cv::COLOR_RGB2BGR;
  } else if (encoding == "rgba8") {
    source_type = CV_8UC4;
    conversion = cv::COLOR_RGBA2BGR;
  } else if (encoding == "bgra8") {
    source_type = CV_8UC4;
    conversion = cv::COLOR_BGRA2BGR;
  } else if (encoding == "mono8" || encoding == "8uc1") {
    source_type = CV_8UC1;
    conversion = cv::COLOR_GRAY2BGR;
  } else {
    throw std::invalid_argument("Unsupported image encoding: " + message.encoding);
  }

  if (message.width == 0 || message.height == 0) {
    throw std::invalid_argument("Invalid image size");
  }

  const size_t channels = static_cast<size_t>(CV_MAT_CN(source_type));
  const size_t minimum_step = static_cast<size_t>(message.width) * channels;
  const size_t step = message.step != 0 ? message.step : minimum_step;
  if (step < minimum_step || message.data.size() < step * message.height) {
    throw std::invalid_argument("Image data is smaller than its declared geometry");
  }

  // Reference the message buffer in place; imencode/cvtColor only read from it.
  const cv::Mat source(
    static_cast<int>(message.height), static_cast<int>(message.width), source_type,
    const_cast<uint8_t *>(message.data.data()), step);

  if (conversion < 0) {
    return source;
  }
  cv::Mat converted;
  cv::cvtColor(source, converted, conversion);
  return converted;
}

}  // namespace

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
      const cv::Mat image = to_bgr8(*message);
      sensor_msgs::msg::CompressedImage output;
      output.header = message->header;
      output.format = "jpeg";
      const std::vector<int> parameters = {
        cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
      if (!cv::imencode(".jpg", image, output.data, parameters)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "JPEG preview encoding failed");
        return;
      }
      output_publisher_->publish(std::move(output));
    } catch (const std::invalid_argument & exception) {
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
