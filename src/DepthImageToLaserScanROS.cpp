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

#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging_macros.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <functional>

#include <depthimage_to_laserscan/DepthImageToLaserScanROS.h>

#define ROS_ERROR RCUTILS_LOG_ERROR
#define ROS_ERROR_THROTTLE(sec, ...) RCUTILS_LOG_ERROR_THROTTLE(RCUTILS_STEADY_TIME, sec, __VA_ARGS__)

using namespace depthimage_to_laserscan;

DepthImageToLaserScanROS::DepthImageToLaserScanROS(rclcpp::Node::SharedPtr & node):node_(node) {

  cam_info_sub_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>("depth_camera_info",
      std::bind(
        &DepthImageToLaserScanROS::infoCb, this,
        std::placeholders::_1),
      rmw_qos_profile_sensor_data);

  depth_image_sub_ =
    node_->create_subscription<sensor_msgs::msg::Image>("depth",
      std::bind(&DepthImageToLaserScanROS::depthCb, this, std::placeholders::_1),
      rmw_qos_profile_sensor_data);

  scan_pub_ = node_->create_publisher<sensor_msgs::msg::LaserScan>("scan");

  float scan_time = 0.033;
  node_->get_parameter("scan_time", scan_time);
  dtl_.set_scan_time(scan_time);

  float range_min = 0.2;
  float range_max = 4.0;
  node_->get_parameter("range_min", range_min);
  node_->get_parameter("range_max", range_max);
  dtl_.set_range_limits(range_min, range_max);

  int scan_height = 1;
  node_->get_parameter("scan_height", scan_height);
  dtl_.set_scan_height(scan_height);

  std::string output_frame = "base_link";
  node_->get_parameter("output_frame", output_frame);
  dtl_.set_output_frame(output_frame);

  std::string depth_optical_frame = "camera_depth_optical_frame";
  node_->get_parameter("depth_optical_frame", depth_optical_frame);
  dtl_.set_depth_optical_frame(depth_optical_frame);


  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(node_->get_logger(), "Transform from  depth_optical_frame to output frame!");
  rclcpp::Time transform_time = node_->now();
  std::string tf_error;
  while (rclcpp::ok() && !tf_buffer_->canTransform(depth_optical_frame, output_frame, tf2_ros::fromMsg(transform_time),
    tf2::durationFromSec(3.0), &tf_error)) //tf2::TimePointZero
  {
    RCLCPP_INFO(node_->get_logger(), "Timed out waiting for transform from %s to %s"
      " to become available, tf error: %s",
      depth_optical_frame.c_str(), output_frame.c_str(), tf_error.c_str());
    tf_error.clear();
  }

  geometry_msgs::msg::TransformStamped tf_depthOpticalFrame2outputFrame;
  try {
    tf_depthOpticalFrame2outputFrame = tf_buffer_->lookupTransform(output_frame, depth_optical_frame, tf2::TimePointZero);
    RCLCPP_INFO(node_->get_logger(), "transform from %s to %s is : x = %.4f; y = %.4f; z = %.4f;", 
                depth_optical_frame.c_str(), 
                output_frame.c_str(),
                tf_depthOpticalFrame2outputFrame.transform.translation.x,
                tf_depthOpticalFrame2outputFrame.transform.translation.y,
                tf_depthOpticalFrame2outputFrame.transform.translation.z);
    dtl_.depthDevice2output_tf_.setOrigin(tf2::Vector3(
        tf_depthOpticalFrame2outputFrame.transform.translation.x,
        tf_depthOpticalFrame2outputFrame.transform.translation.y,
        tf_depthOpticalFrame2outputFrame.transform.translation.z)); 

    dtl_.depthDevice2output_tf_.setRotation(tf2::Quaternion(
        tf_depthOpticalFrame2outputFrame.transform.rotation.x,
        tf_depthOpticalFrame2outputFrame.transform.rotation.y,
        tf_depthOpticalFrame2outputFrame.transform.rotation.z,
        tf_depthOpticalFrame2outputFrame.transform.rotation.w));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(node_->get_logger(), ex.what());
  }
}

DepthImageToLaserScanROS::~DepthImageToLaserScanROS()
{
}

void DepthImageToLaserScanROS::infoCb(sensor_msgs::msg::CameraInfo::SharedPtr info)
{
  cam_info_ = info;
}

void DepthImageToLaserScanROS::depthCb(const sensor_msgs::msg::Image::SharedPtr image)
{
  if (nullptr == cam_info_) {
    ROS_ERROR("No camera info, skipping point cloud squash");
    return;
  }

  try
  {
    sensor_msgs::msg::LaserScan::SharedPtr scan_msg = dtl_.convert_msg(image, cam_info_);
    scan_pub_->publish(scan_msg);
  }
  catch (std::runtime_error& e)
  {
    ROS_ERROR_THROTTLE(1.0, "Could not convert depth image to laserscan: %s", e.what());
  }
}
