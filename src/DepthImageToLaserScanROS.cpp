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

#include <depthimage_to_laserscan/DepthImageToLaserScanROS.h>

using namespace depthimage_to_laserscan;
  
DepthImageToLaserScanROS::DepthImageToLaserScanROS(ros::NodeHandle& n, ros::NodeHandle& pnh):pnh_(pnh), it_(n), srv_(pnh) {
  boost::mutex::scoped_lock lock(connect_mutex_);
  
  // Dynamic Reconfigure
  dynamic_reconfigure::Server<depthimage_to_laserscan::DepthConfig>::CallbackType f;
  f = boost::bind(&DepthImageToLaserScanROS::reconfigureCb, this, _1, _2);
  srv_.setCallback(f);

  // Lazy subscription to depth image topic
  pub_ = n.advertise<sensor_msgs::LaserScan>("scan", 10, boost::bind(&DepthImageToLaserScanROS::connectCb, this, _1), boost::bind(&DepthImageToLaserScanROS::disconnectCb, this, _1));
}

DepthImageToLaserScanROS::~DepthImageToLaserScanROS(){
  sub_.shutdown();
}

void DepthImageToLaserScanROS::depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
	const sensor_msgs::CameraInfoConstPtr& info_msg) {
  try
  {
    tf::StampedTransform depthOpticalTransform;

    // listen to transform only once to avoid overhead
    try
    {
        if (listener_.canTransform("/map", "/camera_depth_optical_frame", ros::Time(0))) {
            listener_.lookupTransform("/map", "/camera_depth_optical_frame", ros::Time(0), depthOpticalTransform);
        } else {
            throw tf::TransformException("Failed to transform /map to /camera_depth_optical_frame");
        }
    }
    catch (tf::TransformException &ex)
    {
        // Number of seconds to wait for static publisher before logging an error
        const ros::Duration opticalFrameTimeout(5, 0);

        if (!firstOpticalFrameTime_.isValid())
        {
            firstOpticalFrameTime_ = ros::Time::now();
        }
        else if ((ros::Time::now() - firstOpticalFrameTime_) > opticalFrameTimeout)
        {
            ROS_ERROR_THROTTLE(5.0, "Depth Image to Laserscan: %s", ex.what());
        }

        return;
    }

    if(lastImageReceivedTime_.isValid())
    {
      double receivingTime = (depth_msg->header.stamp - lastImageReceivedTime_).toSec();
      if(receivingTime > (1.0f / EXPECTED_IMAGE_FREQUENCY + MAX_ALLOWED_IMAGE_DELAY))
      {
        ROS_INFO_THROTTLE(1, "Message receiving time (%f sec) is larger than expected (%f sec)", receivingTime, 1.0 / EXPECTED_IMAGE_FREQUENCY);
      }
    }

    sensor_msgs::LaserScanPtr scan_msg = dtl_.convert_msg(depth_msg, info_msg, depthOpticalTransform);
    pub_.publish(scan_msg);
    lastImageReceivedTime_ = depth_msg->header.stamp;
  }
  catch (std::runtime_error& e)
  {
    ROS_ERROR_THROTTLE(1.0, "Could not convert depth image to laserscan: %s", e.what());
  }
}

void DepthImageToLaserScanROS::connectCb(const ros::SingleSubscriberPublisher& pub) {
  boost::mutex::scoped_lock lock(connect_mutex_);
  if (!sub_ && pub_.getNumSubscribers() > 0) {
    ROS_DEBUG("Connecting to depth topic.");
    image_transport::TransportHints hints("raw", ros::TransportHints(), pnh_);
    sub_ = it_.subscribeCamera("image", 1, &DepthImageToLaserScanROS::depthCb, this, hints);
  }
}

void DepthImageToLaserScanROS::disconnectCb(const ros::SingleSubscriberPublisher& pub) {
  boost::mutex::scoped_lock lock(connect_mutex_);
  if (pub_.getNumSubscribers() == 0) {
    ROS_DEBUG("Unsubscribing from depth topic.");
    sub_.shutdown();
  }
}

void DepthImageToLaserScanROS::reconfigureCb(depthimage_to_laserscan::DepthConfig& config, uint32_t level){
    dtl_.set_scan_time(config.scan_time);
    dtl_.set_range_limits(config.range_min, config.range_max);
    dtl_.set_scan_height(config.scan_height);
    dtl_.set_output_frame(config.output_frame_id);
    dtl_.set_height_limits(config.height_min, config.height_max);
}
