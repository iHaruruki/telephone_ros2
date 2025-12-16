#include <chrono>
#include <memory>
#include <vector>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

#include <portaudio.h>

using namespace std::chrono_literals;

class MicPublisherNode : public rclcpp::Node
{
public:
  MicPublisherNode()
  : Node("mic_publisher_node")
  {
    // パラメータ宣言
    declare_parameter<int>("sample_rate", 16000);
    declare_parameter<int>("channels", 1);
    declare_parameter<int>("chunk_ms", 50);

    sample_rate_ = get_parameter("sample_rate").as_int();
    channels_    = get_parameter("channels").as_int();
    chunk_ms_    = get_parameter("chunk_ms").as_int();

    chunk_samples_ = static_cast<int>(sample_rate_ * chunk_ms_ / 1000);

    publisher_ = create_publisher<std_msgs::msg::Int16MultiArray>(
      "audio_raw", 10);

    RCLCPP_INFO(
      get_logger(),
      "Starting mic publisher (C++): rate=%d Hz, channels=%d, chunk=%d ms (%d samples)",
      sample_rate_, channels_, chunk_ms_, chunk_samples_);

    // PortAudio 初期化
    PaError err = Pa_Initialize();
    if (err != paNoError) {
      RCLCPP_FATAL(get_logger(), "PortAudio initialization failed: %s", Pa_GetErrorText(err));
      throw std::runtime_error("PortAudio initialization failed");
    }

    // 入力パラメータ設定
    input_params_.device = Pa_GetDefaultInputDevice();
    if (input_params_.device == paNoDevice) {
      RCLCPP_FATAL(get_logger(), "No default input device.");
      throw std::runtime_error("No default input device.");
    }

    const PaDeviceInfo *deviceInfo = Pa_GetDeviceInfo(input_params_.device);
    RCLCPP_INFO(get_logger(), "Using input device: %s", deviceInfo->name);

    input_params_.channelCount = channels_;
    input_params_.sampleFormat = paInt16; // 16bit PCM
    input_params_.suggestedLatency = deviceInfo->defaultLowInputLatency;
    input_params_.hostApiSpecificStreamInfo = nullptr;

    // ストリーム作成
    err = Pa_OpenStream(
      &stream_,
      &input_params_,       // input
      nullptr,              // output
      static_cast<double>(sample_rate_),
      static_cast<unsigned long>(chunk_samples_),
      paNoFlag,
      &MicPublisherNode::paCallback,
      this                  // userData
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

    RCLCPP_INFO(get_logger(), "Mic publisher started.");
  }

  ~MicPublisherNode() override
  {
    // ストリーム停止・終了処理
    if (stream_) {
      PaError err = Pa_StopStream(stream_);
      if (err != paNoError) {
        RCLCPP_WARN(get_logger(), "Pa_StopStream failed: %s", Pa_GetErrorText(err));
      }
      Pa_CloseStream(stream_);
      stream_ = nullptr;
    }
    Pa_Terminate();
    RCLCPP_INFO(get_logger(), "Mic publisher stopped.");
  }

  // PortAudio のコールバックから呼び出される静的関数
  static int paCallback(
    const void *inputBuffer,
    void *outputBuffer,
    unsigned long framesPerBuffer,
    const PaStreamCallbackTimeInfo* timeInfo,
    PaStreamCallbackFlags statusFlags,
    void *userData)
  {
    (void)outputBuffer;
    (void)timeInfo;
    (void)statusFlags;

    auto *node = static_cast<MicPublisherNode*>(userData);
    return node->paCallbackImpl(inputBuffer, framesPerBuffer);
  }

  // 実際のコールバック処理
  int paCallbackImpl(const void *inputBuffer, unsigned long framesPerBuffer)
  {
    if (inputBuffer == nullptr) {
      // サイレントバッファ
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Input buffer is null, publishing silence.");
      // 必要なら無音を詰めて publish してもよい
      return paContinue;
    }

    const int16_t *in = static_cast<const int16_t*>(inputBuffer);
    size_t sample_count = static_cast<size_t>(framesPerBuffer * channels_);

    std_msgs::msg::Int16MultiArray msg;
    msg.data.resize(sample_count);
    std::copy(in, in + sample_count, msg.data.begin());

    // 注意：PortAudio のコールバックスレッドから直接 publish している。
    // 高負荷環境やリアルタイム性がシビアな場合はリングバッファ＋タイマコールバックなどで
    // メインスレッド側から publish する設計にした方が安全。
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
  try {
    auto node = std::make_shared<MicPublisherNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    std::cerr << "Exception in MicPublisherNode: " << e.what() << std::endl;
  }
  rclcpp::shutdown();
  return 0;
}