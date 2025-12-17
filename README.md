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

### 1 on 1 mode
#### PC A side
```bash
# 端末 A1: マイク → audio_a_out
ros2 run telephone_ros2 publisher_a_node
```
```bash
# 端末 A2: audio_b_out を再生
ros2 run telephone_ros2 subscriber_a_node 
```
#### PC B side
```bash
# 端末 B1: マイク → audio_b_out
ros2 run telephone_ros2 publisher_b_node
```
```bash
# 端末 B2: audio_a_out を再生
ros2 run telephone_ros2 subscriber_b_node 
```

### Operate on two devices
#### PC A side
Mic → audio_a_out
```bash
# Terminal 1
ros2 run telephone_ros2 audio_publisher_node --ros-args -p topic:=audio_a_out -p sample_rate:=16000 -p channels:=1 -p chunk_ms:=50
```
Play audio
```bash
# Terminal 2
ros2 run telephone_ros2 audio_subscriber_node --ros-args -p topic:=audio_b_out -p sample_rate:=16000 -p channels:=1
```
spectrum_viewer
```bash
ros2 run telephone_ros2 spectrum_viewer_node   --ros-args -p topic:=audio_b_out -p sample_rate:=16000 -p fft_size:=1024
```

#### PC B side
Mic → audio_a_out
```bash
# Terminal 1
ros2 run telephone_ros2 audio_publisher_node --ros-args -p topic:=audio_b_out -p sample_rate:=16000 -p channels:=1 -p chunk_ms:=50
```
Play audio
```bash
# Terminal 2
ros2 run telephone_ros2 audio_subscriber_node --ros-args -p topic:=audio_a_out -p sample_rate:=16000 -p channels:=1
```
spectrum_viewer
```bash
ros2 run telephone_ros2 spectrum_viewer_node   --ros-args -p topic:=audio_a_out -p sample_rate:=16000 -p fft_size:=1024
```

### Operate on three or more devices
#### PC A side
Mic → audio_a_out
```bash
# Terminal 1
ros2 run telephone_ros2 audio_publisher_node --ros-args -p topic:=audio_a_out -p sample_rate:=16000 -p channels:=1 -p chunk_ms:=50
```
Play audio
```bash
# Terminal 2
ros2 run telephone_ros2 audio_player_node --ros-args -p topic:=/telephone/mixed/out -p sample_rate:=16000 -p channels:=1
```
mixer_node
```bash
ros2 run telephone_ros2 mixer_node --ros-args -p input_topics:="[\"/audio_a_out\", \"/audio_b_out\", \"/audio_c_out\"]" -p output_topic:=/telephone/mixed/out -p sample_rate:=16000 -p channels:=1
```

#### PC B side
Mic → audio_a_out
```bash
# Terminal 1
ros2 run telephone_ros2 audio_publisher_node --ros-args -p topic:=audio_b_out -p sample_rate:=16000 -p channels:=1 -p chunk_ms:=50
```
Play audio
```bash
# Terminal 2
ros2 run telephone_ros2 audio_player_node --ros-args -p topic:=/telephone/mixed/out -p sample_rate:=16000 -p channels:=1
```

#### PC C side
Mic → audio_a_out
```bash
# Terminal 1
ros2 run telephone_ros2 audio_publisher_node --ros-args -p topic:=audio_c_out -p sample_rate:=16000 -p channels:=1 -p chunk_ms:=50
```
Play audio
```bash
# Terminal 2
ros2 run telephone_ros2 audio_player_node --ros-args -p topic:=/telephone/mixed/out -p sample_rate:=16000 -p channels:=1
```

## 👤 Authors

- **[iHaruruki](https://github.com/iHaruruki)** — Main author & maintainer

## 📚 References