/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* 
 * Author: Chad Rockey
 */

// Bring in my package's API, which is what I'm testing
#include <depthimage_to_laserscan/DepthImageToLaserScan.h>
// Bring in gtest
#include <gtest/gtest.h>

#include <limits.h>
#include <math.h>

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

// Library object
depthimage_to_laserscan::DepthImageToLaserScan dtl_;

// Inputs
sensor_msgs::ImagePtr depth_msg_;
sensor_msgs::CameraInfoPtr info_msg_;

// Check if the setters work properly and initialize member variables
TEST(ConvertTest, setupLibrary)
{
  // Set up library
  const float scan_time = 1.0/30.0;
  dtl_.set_scan_time(scan_time);
  const float range_min = 0.45;
  const float range_max = 10.0;
  dtl_.set_range_limits(range_min, range_max);
  const int scan_height = 1;
  dtl_.set_scan_height(scan_height);
  const std::string output_frame = "camera_depth_frame";
  dtl_.set_output_frame(output_frame);
  
  depth_msg_.reset(new sensor_msgs::Image);
  depth_msg_->header.seq = 42;
  depth_msg_->header.stamp.fromNSec(1234567890);
  depth_msg_->header.frame_id = "frame";
  depth_msg_->height = 480;
  depth_msg_->width = 640;
  depth_msg_->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  depth_msg_->is_bigendian = false;
  depth_msg_->step = depth_msg_->width*2; // 2 bytes per pixel
  uint16_t value = 0x0F;
  depth_msg_->data.assign(depth_msg_->height*depth_msg_->step, value); // Sets all values to 3.855m
  
  info_msg_.reset(new sensor_msgs::CameraInfo);
  info_msg_->header = depth_msg_->header;
  info_msg_->height = depth_msg_->height;
  info_msg_->width = depth_msg_->width;
  info_msg_->distortion_model = "plumb_bob";
  info_msg_->D.resize(5); // All 0, no distortion
  info_msg_->K[0] = 570.3422241210938;
  info_msg_->K[2] = 314.5;
  info_msg_->K[4] = 570.3422241210938;
  info_msg_->K[5] = 235.5;
  info_msg_->K[8] = 1.0;
  info_msg_->R[0] = 1.0;
  info_msg_->R[4] = 1.0;
  info_msg_->R[8] = 1.0;
  info_msg_->P[0] = 570.3422241210938;
  info_msg_->P[2] = 314.5;
  info_msg_->P[5] = 570.3422241210938;
  info_msg_->P[6] = 235.5;
  info_msg_->P[10] = 1.0;
  
  sensor_msgs::LaserScanPtr scan_msg = dtl_.convert_msg(depth_msg_, info_msg_);
  
  // Test set variables
  EXPECT_EQ(scan_msg->scan_time, scan_time);
  EXPECT_EQ(scan_msg->range_min, range_min);
  EXPECT_EQ(scan_msg->range_max, range_max);
  EXPECT_EQ(scan_msg->header.frame_id, output_frame);
  EXPECT_EQ(scan_msg->ranges.size(), depth_msg_->width);
}

// Test for the exception based on encoding
TEST(ConvertTest, testExceptions)
{
  // Test supported image encodings for exceptions
  // Does not segfault as long as scan_height = 1
  depth_msg_->encoding = sensor_msgs::image_encodings::RGB8;
  EXPECT_THROW(dtl_.convert_msg(depth_msg_, info_msg_), std::runtime_error);
  depth_msg_->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  EXPECT_NO_THROW(dtl_.convert_msg(depth_msg_, info_msg_));
  depth_msg_->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  EXPECT_NO_THROW(dtl_.convert_msg(depth_msg_, info_msg_));
}

// Check to make sure the mininum is output for each pixel column for various scan heights
TEST(ConvertTest, testScanHeight)
{
  for(int scan_height = 1; scan_height <= 100; scan_height++){
    uint16_t low_value = 500;
    uint16_t high_value = 3000;

    int data_len = depth_msg_->width;
    uint16_t* data = reinterpret_cast<uint16_t*>(&depth_msg_->data[0]);
    int row_step = depth_msg_->step / sizeof(uint16_t);

    dtl_.set_scan_height(scan_height);

    int offset = (int)(info_msg_->K[5]-scan_height/2);
    data += offset*row_step; // Offset to center of image

    for(int v = 0; v < scan_height; v++, data += row_step){
      for (int u = 0; u < data_len; u++) // Loop over each pixel in row
      {
	if(v % scan_height == u % scan_height){
	  data[u] = low_value;
	} else {
	  data[u] = high_value;
	}
      }
    }

    // Convert
    sensor_msgs::LaserScanPtr scan_msg = dtl_.convert_msg(depth_msg_, info_msg_);

    // Test for minimum
    float high_float_thresh = (float)high_value * 1.0f/1000.0f * 0.9f; // 0.9f represents 10 percent margin on range
    for(size_t i = 0; i < scan_msg->ranges.size(); i++){
      // If this is a valid point
      if(scan_msg->range_min <= scan_msg->ranges[i] && scan_msg->ranges[i] <= scan_msg->range_max){
	// Make sure it's not set to the high_value
	ASSERT_LT(scan_msg->ranges[i], high_float_thresh);
      }
    }
  }
  
  // Revert to 1 scan height
  dtl_.set_scan_height(1);
  
}

// Test a randomly filled image and ensure all values are < range_min
// (range_max is currently violated to fill the messages)
TEST(ConvertTest, testRandom)
{
  srand ( 8675309 ); // Set seed for repeatable tests
  
  uint16_t* data = reinterpret_cast<uint16_t*>(&depth_msg_->data[0]);
  for(size_t i = 0; i < depth_msg_->width*depth_msg_->height; i++){
    data[i] = rand() % 500; // Distance between 0 and 0.5m
  }
  
  // Convert
  sensor_msgs::LaserScanPtr scan_msg = dtl_.convert_msg(depth_msg_, info_msg_);
  
  // Make sure all values are greater than or equal to range_min and less than or equal to range_max
  for(size_t i = 0; i < scan_msg->ranges.size(); i++){
    if(std::isfinite(scan_msg->ranges[i])){
      ASSERT_GE(scan_msg->ranges[i], scan_msg->range_min);
      ASSERT_LE(scan_msg->ranges[i], scan_msg->range_max);
    }
  }
}

// Test to preserve NaN
TEST(ConvertTest, testNaN)
{
  // Use a floating point image
  sensor_msgs::ImagePtr float_msg(new sensor_msgs::Image(*depth_msg_));
  float_msg->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  float_msg->step = float_msg->width*4; // 4 bytes per pixel
  float_msg->data.resize(float_msg->step * float_msg->height);
  
  float* data = reinterpret_cast<float*>(&float_msg->data[0]);
  for(size_t i = 0; i < float_msg->width*float_msg->height; i++){
    data[i] = std::numeric_limits<float>::quiet_NaN();
  }
  
  // Convert
  sensor_msgs::LaserScanPtr scan_msg = dtl_.convert_msg(float_msg, info_msg_);
  
  // Make sure all values are NaN
  for(size_t i = 0; i < scan_msg->ranges.size(); i++){
    if(std::isfinite(scan_msg->ranges[i])){
      ADD_FAILURE() << "Non-NaN value produced from NaN test.";
    }
  }
}

// Test to preserve +Inf
TEST(ConvertTest, testPositiveInf)
{
  // Use a floating point image
  sensor_msgs::ImagePtr float_msg(new sensor_msgs::Image(*depth_msg_));
  float_msg->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  float_msg->step = float_msg->width*4; // 4 bytes per pixel
  float_msg->data.resize(float_msg->step * float_msg->height);
  
  float* data = reinterpret_cast<float*>(&float_msg->data[0]);
  for(size_t i = 0; i < float_msg->width*float_msg->height; i++){
    data[i] = std::numeric_limits<float>::infinity();
  }
  
  // Convert
  sensor_msgs::LaserScanPtr scan_msg = dtl_.convert_msg(float_msg, info_msg_);
  
  // Make sure most (> 80%) values are Inf
  size_t nan_count = 0;
  for(size_t i = 0; i < scan_msg->ranges.size(); i++){
    if(std::isfinite(scan_msg->ranges[i])){ // NaNs are acceptable.
      ADD_FAILURE() << "Non-finite value produced from postive infniity test.";
    } else if(std::isnan(scan_msg->ranges[i])){
      nan_count++;
    } else if(scan_msg->ranges[i] < 0){
      ADD_FAILURE() << "Negative value produced from postive infinity test.";
    }
  }
  
  ASSERT_LE(nan_count, scan_msg->ranges.size() * 0.80);
}

// Test to preserve -Inf
TEST(ConvertTest, testNegativeInf)
{
  // Use a floating point image
  sensor_msgs::ImagePtr float_msg(new sensor_msgs::Image(*depth_msg_));
  float_msg->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  float_msg->step = float_msg->width*4; // 4 bytes per pixel
  float_msg->data.resize(float_msg->step * float_msg->height);
  
  float* data = reinterpret_cast<float*>(&float_msg->data[0]);
  for(size_t i = 0; i < float_msg->width*float_msg->height; i++){
    data[i] = -std::numeric_limits<float>::infinity();
  }
  
  // Convert
  sensor_msgs::LaserScanPtr scan_msg = dtl_.convert_msg(float_msg, info_msg_);
  
  // Make sure most (> 80%) values are Inf
  size_t nan_count = 0;
  for(size_t i = 0; i < scan_msg->ranges.size(); i++){
    if(std::isfinite(scan_msg->ranges[i])){ // NaNs are acceptable.
      ADD_FAILURE() << "Non-finite value produced from postive infniity test.";
    } else if(std::isnan(scan_msg->ranges[i])){
      nan_count++;
    } else if(scan_msg->ranges[i] > 0){
      ADD_FAILURE() << "Postive value produced from negative infinity test.";
    }
  }
  
  ASSERT_LE(nan_count, scan_msg->ranges.size() * 0.80);
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
testing::InitGoogleTest(&argc, argv);
return RUN_ALL_TESTS();
}
