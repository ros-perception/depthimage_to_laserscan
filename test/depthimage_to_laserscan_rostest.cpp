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

// Bring in gtest
#include <gtest/gtest.h>

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h>

// Outputs
sensor_msgs::ImagePtr depth_msg_;
sensor_msgs::CameraInfoPtr info_msg_;

size_t message_count_;

void callback(const sensor_msgs::LaserScanConstPtr& msg){
  message_count_++;
}

TEST(transform_listener, commsTest)
{
  ros::NodeHandle n;
  
  // Set up output messages
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
  
  // Set up publisher
  ros::Publisher pub = n.advertise<sensor_msgs::Image>("image", 5);
  ros::Publisher pub_info = n.advertise<sensor_msgs::CameraInfo>("camera_info", 5);
  
  // Subscribe to output
  ros::Subscriber sub = n.subscribe("scan", 20, callback);
  
  // Sleep to allow connections to settle
  ros::Duration(5.0).sleep();
  
  size_t number_published = 10;
  double sleep_duration = 1.0/30.0;
  
  // Publish some data
  for(size_t i = 0; i < number_published; i++){
    pub.publish(depth_msg_);
    pub_info.publish(info_msg_);
    ros::Duration(sleep_duration).sleep();
    ros::spinOnce();
  }
  
  // Wait a while longer
  ros::Duration(2.0).sleep();
  ros::spinOnce();
  
  EXPECT_EQ(number_published, message_count_);
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "depthimage_to_laserscan_unittest");

  message_count_ = 0;
  
  return RUN_ALL_TESTS();
}