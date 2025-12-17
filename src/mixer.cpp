#include <memory>
#include <vector>
#include <deque>
#include <string>
#include <mutex>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

class MixerNode : public rclcpp::Node
{
public:
  MixerNode()
  : Node("mixer_node")
  {
    // Parameters
    // 入力トピック一覧（例：["/telephone/alice/out", "/telephone/bob/out", ...]）
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

    RCLCPP_INFO(
      get_logger(),
      "MixerNode: inputs=%zu, output=%s, rate=%d, channels=%d",
      input_topics_.size(), output_topic_.c_str(), sample_rate_, channels_
    );

    // Publisher
    publisher_ = create_publisher<std_msgs::msg::Int16MultiArray>(output_topic_, 10);

    // Resize buffers according to number of inputs
    size_t n_inputs = input_topics_.size();
    buffers_.resize(n_inputs);
    mutexes_.resize(n_inputs);

    // Subscribers
    for (size_t i = 0; i < n_inputs; ++i) {
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
      std::chrono::milliseconds(20),  // 50Hz mix
      std::bind(&MixerNode::mix_and_publish, this)
    );
  }

private:
  void input_callback(size_t idx, const std_msgs::msg::Int16MultiArray::SharedPtr msg)
  {
    if (idx >= buffers_.size()) return;

    std::lock_guard<std::mutex> lock(mutexes_[idx]);
    auto & buf = buffers_[idx];

    // Append incoming samples
    buf.insert(buf.end(), msg->data.begin(), msg->data.end());

    // Limit buffer size to avoid unlimited growth
    const size_t max_samples = sample_rate_ * channels_;  // 1 second buffer
    if (buf.size() > max_samples) {
      buf.erase(buf.begin(), buf.end() - max_samples);
    }
  }

  void mix_and_publish()
  {
    size_t n_inputs = buffers_.size();
    if (n_inputs == 0) return;

    // Determine minimum available samples among all inputs
    size_t min_samples = std::numeric_limits<size_t>::max();
    for (size_t i = 0; i < n_inputs; ++i) {
      std::lock_guard<std::mutex> lock(mutexes_[i]);
      min_samples = std::min(min_samples, buffers_[i].size());
    }

    // Not enough data to mix
    if (min_samples == 0 || min_samples == std::numeric_limits<size_t>::max()) {
      return;
    }

    // Mix at most some chunk size to limit latency
    const size_t max_chunk = static_cast<size_t>(0.02 * sample_rate_) * channels_;  // ~20ms
    size_t mix_samples = std::min(min_samples, max_chunk);

    // Prepare mix buffer (float for headroom)
    std::vector<float> mix(mix_samples, 0.0f);

    // Accumulate from each input
    for (size_t i = 0; i < n_inputs; ++i) {
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

    // Normalize / clip to int16
    std_msgs::msg::Int16MultiArray out_msg;
    out_msg.data.resize(mix_samples);

    for (size_t s = 0; s < mix_samples; ++s) {
      // Simple clipping; you can also divide by number of active inputs to reduce volume
      float v = mix[s];
      if (v > 32767.0f) v = 32767.0f;
      if (v < -32768.0f) v = -32768.0f;
      out_msg.data[s] = static_cast<int16_t>(v);
    }

    publisher_->publish(out_msg);
  }

  // Parameters
  std::vector<std::string> input_topics_;
  std::string output_topic_;
  int sample_rate_;
  int channels_;

  // Buffers per input
  std::vector<std::vector<int16_t>> buffers_;
  std::vector<std::mutex> mutexes_;

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