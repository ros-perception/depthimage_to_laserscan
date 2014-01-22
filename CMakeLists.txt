# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8)
project(depthimage_to_laserscan)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED dynamic_reconfigure image_geometry image_transport nodelet roscpp sensor_msgs)
#find_package(OpenCV REQUIRED)

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(cfg/Depth.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES DepthImageToLaserScan DepthImageToLaserScanROS DepthImageToLaserScanNodelet
  CATKIN_DEPENDS dynamic_reconfigure image_geometry image_transport nodelet roscpp sensor_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(DepthImageToLaserScan src/DepthImageToLaserScan.cpp)
target_link_libraries(DepthImageToLaserScan ${catkin_LIBRARIES})

add_library(DepthImageToLaserScanROS src/DepthImageToLaserScanROS.cpp)
add_dependencies(DepthImageToLaserScanROS ${PROJECT_NAME}_gencfg)
target_link_libraries(DepthImageToLaserScanROS DepthImageToLaserScan ${catkin_LIBRARIES})

add_library(DepthImageToLaserScanNodelet src/DepthImageToLaserScanNodelet.cpp)
target_link_libraries(DepthImageToLaserScanNodelet DepthImageToLaserScanROS ${catkin_LIBRARIES})

add_executable(depthimage_to_laserscan src/depthimage_to_laserscan.cpp)
target_link_libraries(depthimage_to_laserscan DepthImageToLaserScanROS ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  # Test the library
  catkin_add_gtest(libtest test/DepthImageToLaserScanTest.cpp)
  target_link_libraries(libtest DepthImageToLaserScan ${catkin_LIBRARIES})
endif()

# add the test executable, keep it from being built by "make all"
add_executable(test_dtl EXCLUDE_FROM_ALL test/depthimage_to_laserscan_rostest.cpp)

# Install targets
install(TARGETS DepthImageToLaserScan DepthImageToLaserScanROS DepthImageToLaserScanNodelet depthimage_to_laserscan
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
