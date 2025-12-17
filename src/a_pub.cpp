#include <chrono>
#include <memory>
#include <vector>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

#include <portaudio.h>

class MicPublisherANode : public rclcpp::Node
{
public:
  MicPublisherANode()
  : Node("mic_publisher_a_node")
  {
    declare_parameter<int>("sample_rate", 16000);
    declare_parameter<int>("channels", 1);
    declare_parameter<int>("chunk_ms", 50);

    sample_rate_ = get_parameter("sample_rate").as_int();
    channels_    = get_parameter("channels").as_int();
    chunk_ms_    = get_parameter("chunk_ms").as_int();

    chunk_samples_ = static_cast<int>(sample_rate_ * chunk_ms_ / 1000);

    // A の送信用トピック
    publisher_ = create_publisher<std_msgs::msg::Int16MultiArray>(
      "audio_a_out", 10);

    RCLCPP_INFO(
      get_logger(),
      "[A] Mic publisher: rate=%d Hz, channels=%d, chunk=%d ms (%d samples)",
      sample_rate_, channels_, chunk_ms_, chunk_samples_);

    PaError err = Pa_Initialize();
    if (err != paNoError) {
      RCLCPP_FATAL(get_logger(), "PortAudio init failed: %s", Pa_GetErrorText(err));
      throw std::runtime_error("PortAudio init failed");
    }

    input_params_.device = Pa_GetDefaultInputDevice();
    if (input_params_.device == paNoDevice) {
      RCLCPP_FATAL(get_logger(), "No default input device.");
      throw std::runtime_error("No default input device.");
    }
    const PaDeviceInfo *info = Pa_GetDeviceInfo(input_params_.device);
    RCLCPP_INFO(get_logger(), "[A] Using input device: %s", info->name);

    input_params_.channelCount = channels_;
    input_params_.sampleFormat = paInt16;
    input_params_.suggestedLatency = info->defaultLowInputLatency;
    input_params_.hostApiSpecificStreamInfo = nullptr;

    err = Pa_OpenStream(
      &stream_,
      &input_params_,
      nullptr,
      static_cast<double>(sample_rate_),
      static_cast<unsigned long>(chunk_samples_),
      paNoFlag,
      &MicPublisherANode::paCallback,
      this
    );
    if (err != paNoError) {
      RCLCPP_FATAL(get_logger(), "Pa_OpenStream failed: %s", Pa_GetErrorText(err));
      throw std::runtime_error("Pa_OpenStream failed");
    }

    err = Pa_StartStream(stream_);
    if (err != paNoError) {
      RCLCPP_FATAL(get_logger(), "Pa_StartStream failed: %s", Pa_GetErrorText(err));
      throw std::runtime_error("Pa_StartStream failed");
    }

    RCLCPP_INFO(get_logger(), "[A] Mic publisher started.");
  }

  ~MicPublisherANode() override
  {
    if (stream_) {
      Pa_StopStream(stream_);
      Pa_CloseStream(stream_);
      stream_ = nullptr;
    }
    Pa_Terminate();
    RCLCPP_INFO(get_logger(), "[A] Mic publisher stopped.");
  }

  static int paCallback(
    const void *inputBuffer,
    void *outputBuffer,
    unsigned long framesPerBuffer,
    const PaStreamCallbackTimeInfo *timeInfo,
    PaStreamCallbackFlags statusFlags,
    void *userData)
  {
    (void)outputBuffer;
    (void)timeInfo;
    (void)statusFlags;
    auto *node = static_cast<MicPublisherANode*>(userData);
    return node->paCallbackImpl(inputBuffer, framesPerBuffer);
  }

  int paCallbackImpl(const void *inputBuffer, unsigned long framesPerBuffer)
  {
    if (inputBuffer == nullptr) {
      return paContinue;
    }

    const int16_t *in = static_cast<const int16_t*>(inputBuffer);
    size_t sample_count = static_cast<size_t>(framesPerBuffer * channels_);

    std_msgs::msg::Int16MultiArray msg;
    msg.data.assign(in, in + sample_count);
    publisher_->publish(msg);

    return paContinue;
  }

private:
  int sample_rate_;
  int channels_;
  int chunk_ms_;
  int chunk_samples_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
  PaStream *stream_{nullptr};
  PaStreamParameters input_params_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MicPublisherANode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}