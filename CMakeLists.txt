cmake_minimum_required(VERSION 3.5)
project(depthimage_to_laserscan)
if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake_ros REQUIRED)

find_package(image_geometry REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_library(DepthImageToLaserScan
  src/DepthImageToLaserScan.cpp
)
ament_target_dependencies(DepthImageToLaserScan
  "image_geometry"
  "OpenCV"
  "sensor_msgs"
)

add_library(DepthImageToLaserScanROS
  src/DepthImageToLaserScanROS.cpp
)
ament_target_dependencies(DepthImageToLaserScanROS
  "rclcpp"
)
target_link_libraries(DepthImageToLaserScanROS DepthImageToLaserScan)

add_executable(depthimage_to_laserscan_node
  src/depthimage_to_laserscan.cpp
)
target_link_libraries(depthimage_to_laserscan_node
  DepthImageToLaserScan
  DepthImageToLaserScanROS
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS DepthImageToLaserScan DepthImageToLaserScanROS
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS depthimage_to_laserscan_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}-test test/DepthImageToLaserScanTest.cpp)
  target_link_libraries(${PROJECT_NAME}-test DepthImageToLaserScan)
endif()

ament_export_include_directories(include)
ament_export_libraries(DepthImageToLaserScan DepthImageToLaserScanROS)
ament_package()
