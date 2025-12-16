#include <memory>
#include <vector>
#include <deque>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

#include <opencv2/opencv.hpp>
#include <fftw3.h>

class SpectrumViewerNode : public rclcpp::Node
{
public:
  SpectrumViewerNode()
  : Node("spectrum_viewer_node")
  {
    // パラメータ
    declare_parameter<std::string>("topic", "audio_out");
    declare_parameter<int>("sample_rate", 16000);
    declare_parameter<int>("fft_size", 1024);
    declare_parameter<int>("window_size", 2048);
    declare_parameter<int>("update_interval_ms", 50);  // 更新間隔

    topic_        = get_parameter("topic").as_string();
    sample_rate_  = get_parameter("sample_rate").as_int();
    fft_size_     = get_parameter("fft_size").as_int();
    window_size_  = get_parameter("window_size").as_int();
    int update_ms = get_parameter("update_interval_ms").as_int();

    RCLCPP_INFO(
      get_logger(),
      "Spectrum viewer:  topic=%s, sample_rate=%d Hz, fft_size=%d, window_size=%d",
      topic_.c_str(), sample_rate_, fft_size_, window_size_);

    // FFTW3 初期化
    fft_input_  = fftw_alloc_real(fft_size_);
    fft_output_ = fftw_alloc_complex(fft_size_ / 2 + 1);
    fft_plan_   = fftw_plan_dft_r2c_1d(fft_size_, fft_input_, fft_output_, FFTW_ESTIMATE);

    // サブスクライバ
    subscription_ = create_subscription<std_msgs::msg::Int16MultiArray>(
      topic_,
      10,
      std::bind(&SpectrumViewerNode::audio_callback, this, std::placeholders::_1)
    );

    // 定期的に画面更新
    timer_ = create_wall_timer(
      std::chrono::milliseconds(update_ms),
      std::bind(&SpectrumViewerNode::update_display, this)
    );

    // OpenCV ウィンドウ
    cv:: namedWindow("Waveform", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Spectrum", cv:: WINDOW_AUTOSIZE);

    RCLCPP_INFO(get_logger(), "Spectrum viewer started.");
  }

  ~SpectrumViewerNode() override
  {
    fftw_destroy_plan(fft_plan_);
    fftw_free(fft_input_);
    fftw_free(fft_output_);
    cv::destroyAllWindows();
    RCLCPP_INFO(get_logger(), "Spectrum viewer stopped.");
  }

private:
  void audio_callback(const std_msgs::msg::Int16MultiArray:: SharedPtr msg)
  {
    const auto & data = msg->data;
    if (data.empty()) return;

    // バッファに追加
    for (auto sample : data) {
      audio_buffer_.push_back(sample);
    }

    // バッファサイズ制限
    while (static_cast<int>(audio_buffer_.size()) > window_size_) {
      audio_buffer_.pop_front();
    }
  }

  void update_display()
  {
    if (audio_buffer_.size() < static_cast<size_t>(fft_size_)) {
      return;  // データ不足
    }

    // 波形表示
    draw_waveform();

    // スペクトル表示
    draw_spectrum();

    // キー入力チェック（ESC で終了）
    int key = cv::waitKey(1);
    if (key == 27) {  // ESC
      RCLCPP_INFO(get_logger(), "ESC pressed.  Shutting down.. .");
      rclcpp::shutdown();
    }
  }

  void draw_waveform()
  {
    const int width = 800;
    const int height = 200;
    cv::Mat img = cv::Mat::zeros(height, width, CV_8UC3);

    size_t n = audio_buffer_. size();
    if (n < 2) return;

    // 横軸スケール
    double x_scale = static_cast<double>(width) / static_cast<double>(n);
    // 縦軸スケール（-32768 ~ 32767 → height）
    double y_scale = static_cast<double>(height) / 65536.0;
    int y_offset = height / 2;

    for (size_t i = 1; i < n; ++i) {
      int x1 = static_cast<int>((i - 1) * x_scale);
      int y1 = y_offset - static_cast<int>(audio_buffer_[i - 1] * y_scale);
      int x2 = static_cast<int>(i * x_scale);
      int y2 = y_offset - static_cast<int>(audio_buffer_[i] * y_scale);

      // クリッピング
      y1 = std::max(0, std::min(height - 1, y1));
      y2 = std::max(0, std::min(height - 1, y2));

      cv::line(img, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 1);
    }

    cv::imshow("Waveform", img);
  }

  void draw_spectrum()
  {
    // 最新の fft_size_ サンプルを取得
    size_t buffer_size = audio_buffer_.size();
    size_t offset = buffer_size - fft_size_;

    // ハニング窓を適用して FFT 入力に詰める
    for (int i = 0; i < fft_size_; ++i) {
      double hann = 0.5 * (1.0 - std::cos(2.0 * M_PI * i / (fft_size_ - 1)));
      fft_input_[i] = static_cast<double>(audio_buffer_[offset + i]) * hann;
    }

    // FFT 実行
    fftw_execute(fft_plan_);

    // 振幅を計算（dB スケール）
    int spectrum_size = fft_size_ / 2 + 1;
    std::vector<double> magnitude_db(spectrum_size);

    for (int i = 0; i < spectrum_size; ++i) {
      double real = fft_output_[i][0];
      double imag = fft_output_[i][1];
      double mag = std::sqrt(real * real + imag * imag);
      magnitude_db[i] = 20.0 * std::log10(mag + 1e-10);  // ゼロ除算回避
    }

    // 描画
    const int width = 800;
    const int height = 400;
    cv::Mat img = cv::Mat::zeros(height, width, CV_8UC3);

    // 縦軸範囲：-80 dB ~ 40 dB
    double db_min = -80.0;
    double db_max = 40.0;
    double db_range = db_max - db_min;

    // 横軸：0 Hz ~ sample_rate / 2 Hz
    double freq_max = sample_rate_ / 2.0;
    double x_scale = static_cast<double>(width) / static_cast<double>(spectrum_size);

    for (int i = 1; i < spectrum_size; ++i) {
      int x1 = static_cast<int>((i - 1) * x_scale);
      int x2 = static_cast<int>(i * x_scale);

      // dB を画面座標に変換
      double db1 = magnitude_db[i - 1];
      double db2 = magnitude_db[i];

      int y1 = height - static_cast<int>((db1 - db_min) / db_range * height);
      int y2 = height - static_cast<int>((db2 - db_min) / db_range * height);

      // クリッピング
      y1 = std::max(0, std::min(height - 1, y1));
      y2 = std::max(0, std::min(height - 1, y2));

      cv::line(img, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 0, 255), 1);
    }

    // 周波数軸のラベル（簡易版）
    cv::putText(img, "0 Hz", cv::Point(10, height - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    cv::putText(img, std::to_string(static_cast<int>(freq_max)) + " Hz",
                cv::Point(width - 100, height - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

    cv::imshow("Spectrum", img);
  }

  std::string topic_;
  int sample_rate_;
  int fft_size_;
  int window_size_;

  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::deque<int16_t> audio_buffer_;

  // FFTW3
  double *fft_input_;
  fftw_complex *fft_output_;
  fftw_plan fft_plan_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SpectrumViewerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}