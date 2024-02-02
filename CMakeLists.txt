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

find_package(Sophus REQUIRED)


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

ament_target_dependencies(mono rclcpp sensor_msgs cv_bridge ORB_SLAM3 OpenCV Eigen3 Sophus)

target_link_libraries(mono
    ORB_SLAM3::ORB_SLAM3
)

install(TARGETS mono DESTINATION lib/${PROJECT_NAME})

ament_package()
