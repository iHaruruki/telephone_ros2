#include <memory>
#include <vector>
#include <numeric>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

class AudioSubscriberNode : public rclcpp::Node
{
public:
  AudioSubscriberNode()
  : Node("audio_subscriber_node")
  {
    subscription_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
      "audio_raw",
      10,
      std::bind(&AudioSubscriberNode::topic_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Audio subscriber node started. Subscribing to /audio_raw");
  }

private:
  void topic_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
  {
    const auto & data = msg->data;
    auto size = data.size();

    if (size == 0) {
      RCLCPP_WARN(this->get_logger(), "Received empty audio frame");
      return;
    }

    // 簡単な統計情報を出力（RMS の計算など）
    double sum_sq = 0.0;
    for (auto v : data) {
      sum_sq += static_cast<double>(v) * static_cast<double>(v);
    }
    double rms = std::sqrt(sum_sq / static_cast<double>(size));

    RCLCPP_INFO(
      this->get_logger(),
      "Received audio frame: samples=%zu, first=%d, rms=%.1f",
      size,
      data.front(),
      rms
    );
  }

  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AudioSubscriberNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}