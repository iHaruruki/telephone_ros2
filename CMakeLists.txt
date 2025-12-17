cmake_minimum_required(VERSION 3.8)
project(telephone_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# PortAudio の検出
find_path(PORTAUDIO_INCLUDE_DIR portaudio.h)
find_library(PORTAUDIO_LIBRARY portaudio)

# OpenCV を探す（OpenCV4 対応）
find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
  message(STATUS "Found OpenCV version: ${OpenCV_VERSION}")
  message(STATUS "OpenCV include dirs:  ${OpenCV_INCLUDE_DIRS}")
  message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")
else()
  message(FATAL_ERROR "OpenCV not found.  Please install libopencv-dev")
endif()

if(NOT PORTAUDIO_INCLUDE_DIR OR NOT PORTAUDIO_LIBRARY)
  message(FATAL_ERROR "PortAudio not found. Please install portaudio19-dev.")
endif()

# FFTW3
find_library(FFTW3_LIBRARY fftw3)
if(NOT FFTW3_LIBRARY)
  message(FATAL_ERROR "FFTW3 not found. Please install libfftw3-dev.")
endif()

include_directories(
  include
  ${PORTAUDIO_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

# Sending node (microphone → any topic)
add_executable(audio_publisher_node src/audio_pub.cpp)
ament_target_dependencies(audio_publisher_node rclcpp std_msgs)
target_link_libraries(audio_publisher_node ${PORTAUDIO_LIBRARY})

# Receiving node (any topic → speaker)
add_executable(audio_subscriber_node src/audio_sub.cpp)
ament_target_dependencies(audio_subscriber_node rclcpp std_msgs)
target_link_libraries(audio_subscriber_node ${PORTAUDIO_LIBRARY})

# Spectrum viewer node
add_executable(spectrum_viewer_node src/spectrum_viewer.cpp)
ament_target_dependencies(spectrum_viewer_node rclcpp std_msgs)
target_link_libraries(spectrum_viewer_node ${FFTW3_LIBRARY} ${OpenCV_LIBS})

# Mixer node
add_executable(mixer_node src/mixer.cpp)
ament_target_dependencies(mixer_node rclcpp std_msgs)

install(TARGETS
  audio_publisher_node
  audio_subscriber_node
  spectrum_viewer_node
  mixer_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
