cmake_minimum_required(VERSION 3.5)
project(depthimage_to_laserscan)
include(GenerateExportHeader)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)

find_package(image_geometry REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(DepthImageToLaserScan
  src/DepthImageToLaserScan.cpp
)
target_link_libraries(DepthImageToLaserScan PUBLIC
  image_geometry::image_geometry
  opencv_core
  ${sensor_msgs_TARGETS})

generate_export_header(DepthImageToLaserScan EXPORT_FILE_NAME ${PROJECT_NAME}/DepthImageToLaserScan_export.h)
target_include_directories(DepthImageToLaserScan PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

add_library(DepthImageToLaserScanROS
  src/DepthImageToLaserScanROS.cpp
)
target_link_libraries(DepthImageToLaserScanROS PUBLIC
  rclcpp::rclcpp)
target_link_libraries(DepthImageToLaserScanROS PRIVATE
  rclcpp_components::component)

target_link_libraries(DepthImageToLaserScanROS PUBLIC
  DepthImageToLaserScan)
generate_export_header(DepthImageToLaserScanROS EXPORT_FILE_NAME ${PROJECT_NAME}/DepthImageToLaserScanROS_export.h)

rclcpp_components_register_nodes(DepthImageToLaserScanROS
  "depthimage_to_laserscan::DepthImageToLaserScanROS")

add_executable(depthimage_to_laserscan_node
  src/depthimage_to_laserscan.cpp
)
target_link_libraries(depthimage_to_laserscan_node
  DepthImageToLaserScan
  DepthImageToLaserScanROS
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(DIRECTORY
  launch
  cfg
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS DepthImageToLaserScan DepthImageToLaserScanROS
  EXPORT export_${PROJECT_NAME}
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
  DESTINATION include/${PROJECT_NAME}/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}-test test/DepthImageToLaserScanTest.cpp)
  target_link_libraries(${PROJECT_NAME}-test DepthImageToLaserScan)
endif()

# TODO(sloretz) stop exporting old-style CMake variables in the future
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(DepthImageToLaserScan DepthImageToLaserScanROS)

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rclcpp)
ament_export_dependencies(image_geometry)
ament_export_dependencies(OpenCV)
ament_export_dependencies(sensor_msgs)
ament_package()
