#include <memory>
#include <vector>
#include <deque>
#include <string>
#include <mutex>
#include <array>
#include <algorithm>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

class MixerNode : public rclcpp::Node
{
public:
  MixerNode()
  : Node("mixer_node")
  {
    // Parameters
    declare_parameter<std::vector<std::string>>(
      "input_topics",
      std::vector<std::string>{
        "/telephone/alice/out",
        "/telephone/bob/out"
      }
    );
    declare_parameter<std::string>("output_topic", "/telephone/mixed/out");
    declare_parameter<int>("sample_rate", 16000);
    declare_parameter<int>("channels", 1);

    input_topics_ = get_parameter("input_topics").as_string_array();
    output_topic_ = get_parameter("output_topic").as_string();
    sample_rate_  = get_parameter("sample_rate").as_int();
    channels_     = get_parameter("channels").as_int();

    if (input_topics_.size() > MAX_INPUTS) {
      RCLCPP_WARN(
        get_logger(),
        "input_topics has %zu entries, but MAX_INPUTS=%zu. Extra topics will be ignored.",
        input_topics_.size(), MAX_INPUTS);
      input_count_ = MAX_INPUTS;
    } else {
      input_count_ = input_topics_.size();
    }

    RCLCPP_INFO(
      get_logger(),
      "MixerNode: inputs=%zu, output=%s, rate=%d, channels=%d",
      input_count_, output_topic_.c_str(), sample_rate_, channels_
    );

    // Publisher
    publisher_ = create_publisher<std_msgs::msg::Int16MultiArray>(output_topic_, 10);

    // Initialize buffers for each input actually used
    for (size_t i = 0; i < input_count_; ++i) {
      buffers_[i].clear();
      const auto & topic = input_topics_[i];

      auto cb = [this, i](const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
        this->input_callback(i, msg);
      };
      auto sub = create_subscription<std_msgs::msg::Int16MultiArray>(
        topic, 10, cb);
      subscriptions_.push_back(sub);

      RCLCPP_INFO(get_logger(), "Subscribed to input[%zu]: %s", i, topic.c_str());
    }

    // Timer for mixing
    timer_ = create_wall_timer(
      std::chrono::milliseconds(20),  // ~50Hz
      std::bind(&MixerNode::mix_and_publish, this)
    );
  }

private:
  static constexpr size_t MAX_INPUTS = 8;  // maximum number of inputs supported

  void input_callback(size_t idx, const std_msgs::msg::Int16MultiArray::SharedPtr msg)
  {
    if (idx >= input_count_) return;

    std::lock_guard<std::mutex> lock(mutexes_[idx]);
    auto & buf = buffers_[idx];

    // Append incoming samples
    buf.insert(buf.end(), msg->data.begin(), msg->data.end());

    // Limit buffer size (e.g., max 1 second of audio)
    const size_t max_samples = static_cast<size_t>(sample_rate_) * channels_;
    if (buf.size() > max_samples) {
      buf.erase(buf.begin(), buf.end() - max_samples);
    }
  }

  void mix_and_publish()
  {
    if (input_count_ == 0) {
      return;
    }

    // Determine minimum available samples among all inputs
    size_t min_samples = std::numeric_limits<size_t>::max();
    for (size_t i = 0; i < input_count_; ++i) {
      std::lock_guard<std::mutex> lock(mutexes_[i]);
      min_samples = std::min(min_samples, buffers_[i].size());
    }

    // Not enough data
    if (min_samples == 0 || min_samples == std::numeric_limits<size_t>::max()) {
      return;
    }

    // Mix at most some chunk size to limit latency
    const size_t max_chunk =
      static_cast<size_t>(0.02 * static_cast<double>(sample_rate_)) *
      static_cast<size_t>(channels_);  // about 20ms

    size_t mix_samples = std::min(min_samples, max_chunk);

    // Mix buffer in float for headroom
    std::vector<float> mix(mix_samples, 0.0f);

    // Accumulate from each input
    for (size_t i = 0; i < input_count_; ++i) {
      std::lock_guard<std::mutex> lock(mutexes_[i]);
      auto & buf = buffers_[i];

      if (buf.size() < mix_samples) {
        continue;
      }

      for (size_t s = 0; s < mix_samples; ++s) {
        mix[s] += static_cast<float>(buf[s]);
      }

      // Remove consumed samples
      buf.erase(buf.begin(), buf.begin() + mix_samples);
    }

    // Convert back to int16 with clipping
    std_msgs::msg::Int16MultiArray out_msg;
    out_msg.data.resize(mix_samples);

    for (size_t s = 0; s < mix_samples; ++s) {
      float v = mix[s];
      // Simple hard clipping
      if (v > 32767.0f) v = 32767.0f;
      if (v < -32768.0f) v = -32768.0f;
      out_msg.data[s] = static_cast<int16_t>(v);
    }

    publisher_->publish(out_msg);
  }

  // Parameters
  std::vector<std::string> input_topics_;
  std::string output_topic_;
  int sample_rate_{16000};
  int channels_{1};
  size_t input_count_{0};

  // Buffers & mutexes per input
  std::array<std::vector<int16_t>, MAX_INPUTS> buffers_;
  std::array<std::mutex, MAX_INPUTS> mutexes_;

  // ROS interfaces
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr> subscriptions_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MixerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}