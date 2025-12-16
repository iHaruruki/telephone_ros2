#include <memory>
#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <iostream>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

#include <portaudio.h>

class AudioPlayerNode : public rclcpp::Node
{
public:
  AudioPlayerNode()
  : Node("audio_player_node")
  {
    // パラメータ
    declare_parameter<int>("sample_rate", 16000);
    declare_parameter<int>("channels", 1);

    sample_rate_ = get_parameter("sample_rate").as_int();
    channels_    = get_parameter("channels").as_int();

    RCLCPP_INFO(
      get_logger(),
      "Starting audio player (C++): rate=%d Hz, channels=%d",
      sample_rate_, channels_);

    // PortAudio 初期化
    PaError err = Pa_Initialize();
    if (err != paNoError) {
      RCLCPP_FATAL(get_logger(), "PortAudio initialization failed: %s", Pa_GetErrorText(err));
      throw std::runtime_error("PortAudio initialization failed");
    }

    // 出力デバイス設定
    output_params_.device = Pa_GetDefaultOutputDevice();
    if (output_params_.device == paNoDevice) {
      RCLCPP_FATAL(get_logger(), "No default output device.");
      throw std::runtime_error("No default output device.");
    }

    const PaDeviceInfo *deviceInfo = Pa_GetDeviceInfo(output_params_.device);
    RCLCPP_INFO(get_logger(), "Using output device: %s", deviceInfo->name);

    output_params_.channelCount = channels_;
    output_params_.sampleFormat = paInt16; // 16bit PCM
    output_params_.suggestedLatency = deviceInfo->defaultLowOutputLatency;
    output_params_.hostApiSpecificStreamInfo = nullptr;

    // ストリーム作成
    err = Pa_OpenStream(
      &stream_,
      nullptr,              // input
      &output_params_,      // output
      static_cast<double>(sample_rate_),
      paFramesPerBufferUnspecified, // フレームサイズは PortAudio に任せる
      paNoFlag,
      &AudioPlayerNode::paCallback,
      this
    );

    if (err != paNoError) {
      RCLCPP_FATAL(get_logger(), "Pa_OpenStream failed: %s", Pa_GetErrorText(err));
      throw std::runtime_error("Pa_OpenStream failed");
    }

    // ストリーム開始
    err = Pa_StartStream(stream_);
    if (err != paNoError) {
      RCLCPP_FATAL(get_logger(), "Pa_StartStream failed: %s", Pa_GetErrorText(err));
      throw std::runtime_error("Pa_StartStream failed");
    }

    // サブスクライバ作成
    subscription_ = create_subscription<std_msgs::msg::Int16MultiArray>(
      "audio_raw",
      10,
      std::bind(&AudioPlayerNode::audio_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(), "Audio player node started. Subscribing to /audio_raw");
  }

  ~AudioPlayerNode() override
  {
    // 終了処理
    if (stream_) {
      PaError err = Pa_StopStream(stream_);
      if (err != paNoError) {
        RCLCPP_WARN(get_logger(), "Pa_StopStream failed: %s", Pa_GetErrorText(err));
      }
      Pa_CloseStream(stream_);
      stream_ = nullptr;
    }
    Pa_Terminate();
    RCLCPP_INFO(get_logger(), "Audio player stopped.");
  }

  // PortAudioコールバック（静的）
  static int paCallback(
    const void *inputBuffer,
    void *outputBuffer,
    unsigned long framesPerBuffer,
    const PaStreamCallbackTimeInfo* timeInfo,
    PaStreamCallbackFlags statusFlags,
    void *userData)
  {
    (void)inputBuffer;
    (void)timeInfo;
    (void)statusFlags;

    auto *node = static_cast<AudioPlayerNode*>(userData);
    return node->paCallbackImpl(outputBuffer, framesPerBuffer);
  }

  // 実際の PortAudio 出力処理
  int paCallbackImpl(void *outputBuffer, unsigned long framesPerBuffer)
  {
    int16_t *out = static_cast<int16_t*>(outputBuffer);
    size_t samples_needed = static_cast<size_t>(framesPerBuffer * channels_);

    std::unique_lock<std::mutex> lock(mutex_);

    size_t samples_available = audio_buffer_.size();

    if (samples_available < samples_needed) {
      // 足りない場合はある分だけ出力し、残りは無音で埋める
      size_t samples_to_copy = samples_available;
      for (size_t i = 0; i < samples_to_copy; ++i) {
        out[i] = audio_buffer_[i];
      }
      for (size_t i = samples_to_copy; i < samples_needed; ++i) {
        out[i] = 0; // silence
      }
      audio_buffer_.erase(audio_buffer_.begin(), audio_buffer_.begin() + samples_to_copy);
    } else {
      // 必要分コピー
      for (size_t i = 0; i < samples_needed; ++i) {
        out[i] = audio_buffer_[i];
      }
      audio_buffer_.erase(audio_buffer_.begin(), audio_buffer_.begin() + samples_needed);
    }

    return paContinue;
  }

  // ROS のサブスクライバコールバック
  void audio_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
  {
    const auto & data = msg->data;
    if (data.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty audio frame");
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    // 受信したサンプルをバッファの末尾に追加
    audio_buffer_.insert(audio_buffer_.end(), data.begin(), data.end());
  }

private:
  int sample_rate_;
  int channels_;

  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr subscription_;

  PaStream *stream_{nullptr};
  PaStreamParameters output_params_;

  std::vector<int16_t> audio_buffer_;  // 再生用リングバッファ的なもの（簡易版）
  std::mutex mutex_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<AudioPlayerNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    std::cerr << "Exception in AudioPlayerNode: " << e.what() << std::endl;
  }
  rclcpp::shutdown();
  return 0;
}