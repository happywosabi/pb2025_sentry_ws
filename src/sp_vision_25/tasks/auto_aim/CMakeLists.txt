cmake_minimum_required(VERSION 3.16)

set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")

find_package(OpenVINO REQUIRED COMPONENTS Runtime)

# Try to find ament_index_cpp (optional, for ROS2 builds)
find_package(ament_index_cpp QUIET)

add_subdirectory(planner/tinympc)

add_library(auto_aim OBJECT
    armor.cpp
    classifier.cpp
    detector.cpp
    solver.cpp
    aimer.cpp
    target.cpp
    tracker.cpp
    voter.cpp
    shooter.cpp
    yolo.cpp
    yolos/yolov5.cpp
    yolos/yolov8.cpp
    yolos/yolo11.cpp
    multithread/commandgener.cpp
    multithread/mt_detector.cpp
    planner/planner.cpp
)

target_link_libraries(auto_aim io openvino::runtime tinympcstatic)

# If ament_index_cpp is available, link it
if(ament_index_cpp_FOUND)
  target_link_libraries(auto_aim ament_index_cpp::ament_index_cpp)
  message(STATUS "auto_aim: Building with ROS2 support (ament_index_cpp found)")
else()
  message(STATUS "auto_aim: Building without ROS2 support (ament_index_cpp not found)")
endif()