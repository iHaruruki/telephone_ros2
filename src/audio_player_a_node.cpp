#include <memory>
#include <vector>
#include <mutex>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

#include <portaudio.h>

class AudioPlayerANode : public rclcpp::Node
{
public:
  AudioPlayerANode()
  : Node("audio_player_a_node")
  {
    declare_parameter<int>("sample_rate", 16000);
    declare_parameter<int>("channels", 1);

    sample_rate_ = get_parameter("sample_rate").as_int();
    channels_    = get_parameter("channels").as_int();

    RCLCPP_INFO(
      get_logger(),
      "[A] Audio player: rate=%d Hz, channels=%d",
      sample_rate_, channels_);

    PaError err = Pa_Initialize();
    if (err != paNoError) {
      RCLCPP_FATAL(get_logger(), "PortAudio init failed: %s", Pa_GetErrorText(err));
      throw std::runtime_error("PortAudio init failed");
    }

    output_params_.device = Pa_GetDefaultOutputDevice();
    if (output_params_.device == paNoDevice) {
      RCLCPP_FATAL(get_logger(), "No default output device.");
      throw std::runtime_error("No default output device.");
    }
    const PaDeviceInfo *info = Pa_GetDeviceInfo(output_params_.device);
    RCLCPP_INFO(get_logger(), "[A] Using output device: %s", info->name);

    output_params_.channelCount = channels_;
    output_params_.sampleFormat = paInt16;
    output_params_.suggestedLatency = info->defaultLowOutputLatency;
    output_params_.hostApiSpecificStreamInfo = nullptr;

    err = Pa_OpenStream(
      &stream_,
      nullptr,
      &output_params_,
      static_cast<double>(sample_rate_),
      paFramesPerBufferUnspecified,
      paNoFlag,
      &AudioPlayerANode::paCallback,
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

    // B が送った音声トピックを購読
    subscription_ = create_subscription<std_msgs::msg::Int16MultiArray>(
      "audio_b_out",
      10,
      std::bind(&AudioPlayerANode::audio_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(), "[A] Audio player started. Subscribing to /audio_b_out");
  }

  ~AudioPlayerANode() override
  {
    if (stream_) {
      Pa_StopStream(stream_);
      Pa_CloseStream(stream_);
      stream_ = nullptr;
    }
    Pa_Terminate();
    RCLCPP_INFO(get_logger(), "[A] Audio player stopped.");
  }

  static int paCallback(
    const void *inputBuffer,
    void *outputBuffer,
    unsigned long framesPerBuffer,
    const PaStreamCallbackTimeInfo *timeInfo,
    PaStreamCallbackFlags statusFlags,
    void *userData)
  {
    (void)inputBuffer;
    (void)timeInfo;
    (void)statusFlags;
    auto *node = static_cast<AudioPlayerANode*>(userData);
    return node->paCallbackImpl(outputBuffer, framesPerBuffer);
  }

  int paCallbackImpl(void *outputBuffer, unsigned long framesPerBuffer)
  {
    int16_t *out = static_cast<int16_t*>(outputBuffer);
    size_t samples_needed = static_cast<size_t>(framesPerBuffer * channels_);

    std::lock_guard<std::mutex> lock(mutex_);
    size_t available = audio_buffer_.size();

    if (available < samples_needed) {
      size_t copy_n = available;
      for (size_t i = 0; i < copy_n; ++i) {
        out[i] = audio_buffer_[i];
      }
      for (size_t i = copy_n; i < samples_needed; ++i) {
        out[i] = 0;
      }
      audio_buffer_.erase(audio_buffer_.begin(), audio_buffer_.begin() + copy_n);
    } else {
      for (size_t i = 0; i < samples_needed; ++i) {
        out[i] = audio_buffer_[i];
      }
      audio_buffer_.erase(audio_buffer_.begin(), audio_buffer_.begin() + samples_needed);
    }

    return paContinue;
  }

  void audio_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
  {
    const auto & data = msg->data;
    if (data.empty()) return;

    std::lock_guard<std::mutex> lock(mutex_);
    audio_buffer_.insert(audio_buffer_.end(), data.begin(), data.end());
  }

private:
  int sample_rate_;
  int channels_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr subscription_;
  PaStream *stream_{nullptr};
  PaStreamParameters output_params_;
  std::vector<int16_t> audio_buffer_;
  std::mutex mutex_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AudioPlayerANode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}