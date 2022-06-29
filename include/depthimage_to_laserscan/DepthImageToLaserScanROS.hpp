// Copyright (c) 2012, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/*
 * Author: Chad Rockey
 */

#ifndef DEPTHIMAGE_TO_LASERSCAN__DEPTHIMAGETOLASERSCANROS_HPP_
#define DEPTHIMAGE_TO_LASERSCAN__DEPTHIMAGETOLASERSCANROS_HPP_

#include <memory>

#include "depthimage_to_laserscan/DepthImageToLaserScanROS_export.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <depthimage_to_laserscan/DepthImageToLaserScan.hpp>

namespace depthimage_to_laserscan
{
class DEPTHIMAGETOLASERSCANROS_EXPORT DepthImageToLaserScanROS final : public rclcpp::Node
{
public:
  explicit DepthImageToLaserScanROS(const rclcpp::NodeOptions & options);

  ~DepthImageToLaserScanROS() override;

private:
  /**
   * Callback for image_transport
   *
   * Callback for depth image.  Publishes laserscan at the end of this callback.
   *
   * @param image Image provided by image_transport.
   *
   */
  void depthCb(const sensor_msgs::msg::Image::SharedPtr image);

  void infoCb(sensor_msgs::msg::CameraInfo::SharedPtr info);

  sensor_msgs::msg::CameraInfo::SharedPtr cam_info_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

  ///< Instance of the DepthImageToLaserScan conversion class.
  std::unique_ptr<depthimage_to_laserscan::DepthImageToLaserScan> dtl_;
};
}  // namespace depthimage_to_laserscan

#endif  // DEPTHIMAGE_TO_LASERSCAN__DEPTHIMAGETOLASERSCANROS_HPP_
