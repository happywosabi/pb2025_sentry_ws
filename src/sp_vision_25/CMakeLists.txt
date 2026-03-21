cmake_minimum_required(VERSION 3.16.3)

project(sp_vision_25)

set(CMAKE_CXX_STANDARD 17)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "--------------------CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}--------------------")

# ament_cmake build system
find_package(ament_cmake REQUIRED)

# External dependencies
find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake/")
find_package(OpenVINO REQUIRED)

# ROS2 dependencies for sentry_ros2
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(pb_rm_interfaces REQUIRED)
find_package(auto_aim_interfaces REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})

# Build subdirectory libraries
add_subdirectory(tools)
add_subdirectory(io)
add_subdirectory(tasks/auto_aim)
add_subdirectory(tasks/auto_buff)
add_subdirectory(tasks/omniperception)

####################Primary ROS2 Node###################
# sentry_ros2: Main node for pb2025 integration (native camera mode)
add_executable(sentry_ros2
  src/sentry_ros2.cpp
  io/ros2cboard.cpp)

target_link_libraries(sentry_ros2
  ${OpenCV_LIBS}
  fmt::fmt
  yaml-cpp
  auto_aim
  tools
  io)  # io includes native camera (HikRobot SDK + libusb)

ament_target_dependencies(sentry_ros2
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  pb_rm_interfaces
  auto_aim_interfaces
  example_interfaces
  tf2_geometry_msgs
  ament_index_cpp)

####################Additional Standalone Programs (Optional)###################
# These are kept for development, testing, and calibration purposes
# They don't depend on ROS2

add_executable(standard src/standard.cpp)
add_executable(mt_standard src/mt_standard.cpp)
add_executable(standard_mpc src/standard_mpc.cpp)
add_executable(auto_aim_debug_mpc src/auto_aim_debug_mpc.cpp)
add_executable(mt_auto_aim_debug src/mt_auto_aim_debug.cpp)
add_executable(auto_buff_debug src/auto_buff_debug.cpp)
add_executable(auto_buff_debug_mpc src/auto_buff_debug_mpc.cpp)
add_executable(uav src/uav.cpp)
add_executable(uav_debug src/uav_debug.cpp)

target_link_libraries(standard ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(mt_standard ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(standard_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(auto_aim_debug_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(mt_auto_aim_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(auto_buff_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_buff tools io ${CERES_LIBRARIES})
target_link_libraries(auto_buff_debug_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_buff tools io ${CERES_LIBRARIES})
target_link_libraries(uav ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(uav_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

##################Calibration Tools################
add_executable(capture calibration/capture.cpp)
add_executable(calibrate_camera calibration/calibrate_camera.cpp)
add_executable(calibrate_handeye calibration/calibrate_handeye.cpp)
add_executable(calibrate_robotworld_handeye calibration/calibrate_robotworld_handeye.cpp)
add_executable(split_video calibration/split_video.cpp)
add_executable(capture_no_can calibration/capture_no_can.cpp)
add_executable(capture_ros2 calibration/capture_ros2.cpp)

target_link_libraries(capture ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(capture_no_can ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
target_link_libraries(capture_ros2 ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
ament_target_dependencies(capture_ros2 rclcpp sensor_msgs)
target_link_libraries(calibrate_camera ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(calibrate_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(calibrate_robotworld_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(split_video ${OpenCV_LIBS} fmt::fmt tools)

##################Test Programs##################
add_executable(auto_aim_test tests/auto_aim_test.cpp)
add_executable(auto_buff_test tests/auto_buff_test.cpp)
add_executable(camera_detect_test tests/camera_detect_test.cpp)
add_executable(camera_test tests/camera_test.cpp)
add_executable(camera_thread_test tests/camera_thread_test.cpp)
add_executable(cboard_test tests/cboard_test.cpp)
add_executable(fire_test tests/fire_test.cpp)
add_executable(detector_video_test tests/detector_video_test.cpp)
add_executable(gimbal_response_test tests/gimbal_response_test.cpp)
add_executable(multi_usbcamera_test tests/multi_usbcamera_test.cpp)
add_executable(usbcamera_detect_test tests/usbcamera_detect_test.cpp)
add_executable(usbcamera_test tests/usbcamera_test.cpp)
add_executable(handeye_test tests/handeye_test.cpp)
add_executable(dm_test tests/dm_test.cpp)
add_executable(minimum_vision_system tests/minimum_vision_system.cpp)
add_executable(gimbal_test tests/gimbal_test.cpp)
add_executable(planner_test tests/planner_test.cpp)
add_executable(planner_test_offline tests/planner_test_offline.cpp)

target_link_libraries(auto_aim_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(auto_buff_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_buff tools io)
target_link_libraries(camera_detect_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools auto_aim io)
target_link_libraries(camera_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(camera_thread_test ${OpenCV_LIBS} fmt::fmt auto_aim tools io)
target_link_libraries(cboard_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(fire_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(detector_video_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools auto_aim)
target_link_libraries(gimbal_response_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
target_link_libraries(multi_usbcamera_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(usbcamera_detect_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(usbcamera_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
target_link_libraries(handeye_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(dm_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
target_link_libraries(minimum_vision_system ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(gimbal_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)
target_link_libraries(planner_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)
target_link_libraries(planner_test_offline ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

# Legacy sentry programs (conditionally built if sp_msgs available)
find_package(sp_msgs QUIET)
find_package(std_msgs QUIET)
find_package(rosidl_typesupport_cpp QUIET)
if(sp_msgs_FOUND AND std_msgs_FOUND AND rosidl_typesupport_cpp_FOUND)
    message(STATUS "sp_msgs found, building legacy sentry programs")
    add_executable(sentry src/sentry.cpp)
    add_executable(sentry_bp src/sentry_bp.cpp)
    add_executable(sentry_debug src/sentry_debug.cpp)
    add_executable(sentry_multithread src/sentry_multithread.cpp)
    add_executable(publish_test tests/publish_test.cpp)
    add_executable(subscribe_test tests/subscribe_test.cpp)
    add_executable(topic_loop_test tests/topic_loop_test.cpp)

    target_link_libraries(sentry ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
    target_link_libraries(sentry_bp ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
    target_link_libraries(sentry_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
    target_link_libraries(sentry_multithread ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
    target_link_libraries(publish_test ${OpenCV_LIBS} fmt::fmt tools io auto_aim)
    target_link_libraries(subscribe_test ${OpenCV_LIBS} fmt::fmt tools io)
    target_link_libraries(topic_loop_test ${OpenCV_LIBS} fmt::fmt tools io)

    # Install legacy programs
    install(TARGETS sentry sentry_bp sentry_debug sentry_multithread
            publish_test subscribe_test topic_loop_test
      DESTINATION lib/${PROJECT_NAME})
else()
    message(STATUS "sp_msgs not found, skipping legacy sentry programs")
endif()

####################Install Rules###################
# Install primary ROS2 node
install(TARGETS sentry_ros2
  DESTINATION lib/${PROJECT_NAME})

# Install other executables (for development and testing)
install(TARGETS
  standard mt_standard standard_mpc auto_aim_debug_mpc mt_auto_aim_debug
  auto_buff_debug auto_buff_debug_mpc uav uav_debug
  capture capture_no_can capture_ros2 calibrate_camera calibrate_handeye calibrate_robotworld_handeye split_video
  auto_aim_test auto_buff_test camera_detect_test camera_test camera_thread_test
  cboard_test fire_test detector_video_test gimbal_response_test multi_usbcamera_test
  usbcamera_detect_test usbcamera_test handeye_test dm_test minimum_vision_system
  gimbal_test planner_test planner_test_offline
  DESTINATION lib/${PROJECT_NAME})

# Install configuration files
install(DIRECTORY configs/
  DESTINATION share/${PROJECT_NAME}/configs)

# Install assets (models and resources)
install(DIRECTORY assets/
  DESTINATION share/${PROJECT_NAME}/assets)

# Install models (legacy, kept for compatibility)
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
  OPTIONAL)

# Install launch files (will be created later)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  OPTIONAL)

# Install documentation
install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

# Export package information
ament_package()
