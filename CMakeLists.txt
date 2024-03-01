cmake_minimum_required(VERSION 3.5)
project(orbslam3)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions(-DLITE)

# dsiable all warnings
add_compile_options(-w)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(Sophus REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ORB_SLAM3 REQUIRED)

find_package(DBoW2 REQUIRED)
find_package(g2o REQUIRED)
find_package(Sophus REQUIRED)
find_package(tf2_ros REQUIRED)


include_directories(
  include
#   ${ORB_SLAM3_ROOT_DIR}/include
#   ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
  ${EIGEN3_INCLUDE_DIR}
)

link_directories(
  include
)

add_executable(mono
  src/monocular/mono.cpp
  src/monocular/monocular-slam-node.cpp
)
add_executable(rgbd
  src/rgbd/rgbd.cpp
  src/rgbd/rgbd-slam-node.cpp
)
add_executable(stereo
  src/stereo/stereo.cpp
  src/stereo/stereo-slam-node.cpp
)
add_executable(stereo-inertial
  src/stereo-inertial/stereo-inertial.cpp
  src/stereo-inertial/stereo-inertial-node.cpp
)

target_link_libraries(mono
    ORB_SLAM3::ORB_SLAM3
    DBoW2::DBoW2
    g2o::g2o
)
target_link_libraries(rgbd
    ORB_SLAM3::ORB_SLAM3
    DBoW2::DBoW2
    g2o::g2o
)
target_link_libraries(stereo
    ORB_SLAM3::ORB_SLAM3
    DBoW2::DBoW2
    g2o::g2o
)
target_link_libraries(stereo-inertial
    ORB_SLAM3::ORB_SLAM3
    DBoW2::DBoW2
    g2o::g2o
)

ament_target_dependencies(mono rclcpp sensor_msgs cv_bridge ORB_SLAM3 tf2_ros OpenCV Eigen3)
ament_target_dependencies(rgbd rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM3 tf2_ros OpenCV)
ament_target_dependencies(stereo rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM3 tf2_ros OpenCV Eigen3)
ament_target_dependencies(stereo-inertial rclcpp sensor_msgs cv_bridge ORB_SLAM3 tf2_ros OpenCV)


install(TARGETS mono rgbd stereo stereo-inertial DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config vocabulary DESTINATION share/${PROJECT_NAME}/)

ament_package()
