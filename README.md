# 🤖 lucia_controller
[![ROS 2 Distro - Humble](https://img.shields.io/badge/ros2-Humble-blue)](https://docs.ros.org/en/humble/)

## 🚀 Overview

## 🛠️ Setup
Install library
```bash
sudo apt update
sudo apt install portaudio19-dev libfftw3-dev libopencv-dev
```
clone
```bash
cd ~/ros2_ws/src
git clone https://github.com/iHaruruki/telephone_ros2.git
```
Build
```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select telephone_ros2
source install/setup.bash
```

## 🎮 Usage
### Prerequisites
- Match ROS_DOMAIN_ID on both PCs (e.g., export ROS_DOMAIN_ID=30)  
- Use the same sample rate and channel settings

### PC A side
Mic → audio_a_out
```bash
# Terminal 1
ros2 run audio_mic_pub_cpp mic_publisher_node --ros-args -p topic:=audio_a_out -p sample_rate:=16000 -p channels:=1 -p chunk_ms:=50
```
Play audio
```bash
# Terminal 2
ros2 run audio_mic_pub_cpp audio_player_node --ros-args -p topic:=audio_b_out -p sample_rate:=16000 -p channels:=1
```
spectrum_viewer
```bash
ros2 run telephone_ros2 spectrum_viewer_node   --ros-args -p topic:=audio_b_out -p sample_rate:=16000 -p fft_size:=1024
```

### PC B side
Mic → audio_a_out
```bash
# Terminal 1
ros2 run audio_mic_pub_cpp mic_publisher_node --ros-args -p topic:=audio_b_out -p sample_rate:=16000 -p channels:=1 -p chunk_ms:=50
```
Play audio
```bash
# Terminal 2
ros2 run audio_mic_pub_cpp audio_player_node --ros-args -p topic:=audio_a_out -p sample_rate:=16000 -p channels:=1
```
spectrum_viewer
```bash
ros2 run telephone_ros2 spectrum_viewer_node   --ros-args -p topic:=audio_a_out -p sample_rate:=16000 -p fft_size:=1024
```

## 👤 Authors

- **[iHaruruki](https://github.com/iHaruruki)** — Main author & maintainer

## 📚 References