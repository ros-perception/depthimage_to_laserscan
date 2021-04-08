cmake_minimum_required(VERSION 3.5)
project(depthimage_to_laserscan)
include(GenerateExportHeader)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)

find_package(image_geometry REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(DepthImageToLaserScan
  src/DepthImageToLaserScan.cpp
)
ament_target_dependencies(DepthImageToLaserScan
  "image_geometry"
  "OpenCV"
  "sensor_msgs"
)
generate_export_header(DepthImageToLaserScan EXPORT_FILE_NAME ${PROJECT_NAME}/DepthImageToLaserScan_export.h)
target_include_directories(DepthImageToLaserScan PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

add_library(DepthImageToLaserScanROS
  src/DepthImageToLaserScanROS.cpp
)
ament_target_dependencies(DepthImageToLaserScanROS
  "rclcpp"
  "rclcpp_components"
)
target_link_libraries(DepthImageToLaserScanROS DepthImageToLaserScan)
generate_export_header(DepthImageToLaserScanROS EXPORT_FILE_NAME ${PROJECT_NAME}/DepthImageToLaserScanROS_export.h)
target_include_directories(DepthImageToLaserScanROS PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

rclcpp_components_register_nodes(DepthImageToLaserScanROS
  "depthimage_to_laserscan::DepthImageToLaserScanROS")

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

install(DIRECTORY
  launch
  cfg
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS DepthImageToLaserScan DepthImageToLaserScanROS
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS depthimage_to_laserscan_node
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/DepthImageToLaserScan_export.h
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/DepthImageToLaserScanROS_export.h
  DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}-test test/DepthImageToLaserScanTest.cpp)
  target_link_libraries(${PROJECT_NAME}-test DepthImageToLaserScan)
endif()

ament_export_include_directories(include)
ament_export_libraries(DepthImageToLaserScan DepthImageToLaserScanROS)
ament_package()
