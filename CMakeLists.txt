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

if(NOT PORTAUDIO_INCLUDE_DIR OR NOT PORTAUDIO_LIBRARY)
  message(FATAL_ERROR "PortAudio not found. Please install portaudio19-dev.")
endif()

include_directories(
  include
  ${PORTAUDIO_INCLUDE_DIR}
)

# Sending node (microphone → any topic)
add_executable(audio_publisher_node src/audio_pub.cpp)
ament_target_dependencies(audio_publisher_node rclcpp std_msgs)
target_link_libraries(audio_publisher_node ${PORTAUDIO_LIBRARY})

# Receiving node (any topic → speaker)
add_executable(audio_subscriber_node src/audio_sub.cpp)
ament_target_dependencies(audio_subscriber_node rclcpp std_msgs)
target_link_libraries(audio_subscriber_node ${PORTAUDIO_LIBRARY})


install(TARGETS
  audio_publisher_node
  audio_subscriber_node
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
