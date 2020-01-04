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

#ifndef DEPTH_IMAGE_TO_LASERSCAN_ROS
#define DEPTH_IMAGE_TO_LASERSCAN_ROS

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/thread/mutex.hpp>
#include <dynamic_reconfigure/server.h>
#include <depthimage_to_laserscan/DepthConfig.h>

#include <depthimage_to_laserscan/DepthImageToLaserScan.h>

namespace depthimage_to_laserscan
{
  class DepthImageToLaserScanROS
  {
  public:
    DepthImageToLaserScanROS(ros::NodeHandle& n, ros::NodeHandle& pnh);

    ~DepthImageToLaserScanROS();

  private:
    /**
     * Callback for image_transport
     *
     * Synchronized callback for depth image and camera info.  Publishes laserscan at the end of this callback.
     *
     * @param depth_msg Image provided by image_transport.
     * @param info_msg CameraInfo provided by image_transport.
     *
     */
    void depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
		  const sensor_msgs::CameraInfoConstPtr& info_msg);

    /**
     * Callback that is called when there is a new subscriber.
     *
     * Will not subscribe to the image and camera_info until we have a subscriber for our LaserScan (lazy subscribing).
     *
     */
    void connectCb(const ros::SingleSubscriberPublisher& pub);

    /**
     * Callback called when a subscriber unsubscribes.
     *
     * If all current subscribers of our LaserScan stop listening, stop subscribing (lazy subscribing).
     *
     */
    void disconnectCb(const ros::SingleSubscriberPublisher& pub);

    /**
     * Dynamic reconfigure callback.
     *
     * Callback that is used to set each of the parameters insde the DepthImageToLaserScan object.
     *
     * @param config Dynamic Reconfigure object.
     * @param level Dynamic Reconfigure level.
     *
     */
    void reconfigureCb(depthimage_to_laserscan::DepthConfig& config, uint32_t level);

    ros::NodeHandle pnh_; ///< Private nodehandle used to generate the transport hints in the connectCb.
    image_transport::ImageTransport it_; ///< Subscribes to synchronized Image CameraInfo pairs.
    image_transport::CameraSubscriber sub_; ///< Subscriber for image_transport
    ros::Publisher pub_; ///< Publisher for output LaserScan messages
    dynamic_reconfigure::Server<depthimage_to_laserscan::DepthConfig> srv_; ///< Dynamic reconfigure server

    depthimage_to_laserscan::DepthImageToLaserScan dtl_; ///< Instance of the DepthImageToLaserScan conversion class.

    boost::mutex connect_mutex_; ///< Prevents the connectCb and disconnectCb from being called until everything is initialized.
  };


}; // depthimage_to_laserscan

#endif
