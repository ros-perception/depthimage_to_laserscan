cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

rosbuild_add_library(DepthImageToLaserScan src/DepthImageToLaserScan.cpp)

rosbuild_add_library(DepthImageToLaserScanROS src/DepthImageToLaserScanROS.cpp)
target_link_libraries(DepthImageToLaserScanROS DepthImageToLaserScan)

rosbuild_add_library(DepthImageToLaserScanNodelet src/DepthImageToLaserScanNodelet.cpp)
target_link_libraries(DepthImageToLaserScanNodelet DepthImageToLaserScanROS)

rosbuild_add_executable(depthimage_to_laserscan src/depthimage_to_laserscan.cpp)
target_link_libraries(depthimage_to_laserscan DepthImageToLaserScanROS)

# Test the library
rosbuild_add_gtest(libtest test/DepthImageToLaserScanTest.cpp)
target_link_libraries(libtest DepthImageToLaserScan)

# add the test executable, keep it from being built by "make all"
rosbuild_add_executable(test_dtl EXCLUDE_FROM_ALL test/depthimage_to_laserscan_rostest.cpp)

# Link test against gtest and add a dependency to the "test" target
rosbuild_add_gtest_build_flags(test_dtl)

# Make sure rostest launches during "make test"
rosbuild_add_rostest(test/rostest/depthimage_to_laserscan.test)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
