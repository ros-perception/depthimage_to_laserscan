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

#ifndef DEPTH_IMAGE_TO_LASERSCAN
#define DEPTH_IMAGE_TO_LASERSCAN

#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <image_geometry/pinhole_camera_model.h>
#include <depthimage_to_laserscan/depth_traits.h>
#include <sstream>
#include <limits.h>
#include <math.h>

namespace depthimage_to_laserscan
{ 
  class DepthImageToLaserScan
  {
  public:
    DepthImageToLaserScan();
    ~DepthImageToLaserScan();

    /**
     * Converts the information in a depth image (sensor_msgs::Image) to a sensor_msgs::LaserScan.
     * 
     * This function converts the information in the depth encoded image (UInt16 or Float32 encoding) into
     * a sensor_msgs::LaserScan as accurately as possible.  To do this, it requires the synchornized Image/CameraInfo
     * pair associated with the image.
     * 
     * @param depth_msg UInt16 or Float32 encoded depth image.
     * @param info_msg CameraInfo associated with depth_msg
     * @param depthOpticalTransform The transform from /map to /camera_depth_optical_frame
     * @return sensor_msgs::LaserScanPtr for the center row(s) of the depth image.
     *
     */
    sensor_msgs::LaserScanPtr convert_msg(const sensor_msgs::ImageConstPtr& depth_msg,
	    const sensor_msgs::CameraInfoConstPtr& info_msg, const tf::StampedTransform& depthOpticalTransform);
    
    /**
     * Sets the scan time parameter.
     * 
     * This function stores the desired value for scan_time.  In sensor_msgs::LaserScan, scan_time is defined as 
     * "time between scans [seconds]".  This value is not easily calculated from consquetive messages, and is thus
     * left to the user to set correctly.
     * 
     * @param scan_time The value to use for outgoing sensor_msgs::LaserScan.
     * 
     */
    void set_scan_time(const float scan_time);
    
    /**
     * Sets the minimum and maximum range for the sensor_msgs::LaserScan.
     * 
     * range_min is used to determine how close of a value to allow through when multiple radii correspond to the same
     * angular increment.  range_max is used to set the output message.
     * 
     * @param range_min Minimum range to assign points to the laserscan, also minimum range to use points in the output scan.
     * @param range_max Maximum range to use points in the output scan.
     * 
     */
    void set_range_limits(const float range_min, const float range_max);
    
    /**
     * Sets the number of image rows to use in the output LaserScan.
     * 
     * scan_height is the number of rows (pixels) to use in the output.  This will provide scan_height number of radii for each
     * angular increment.  The output scan will output the closest radius that is still not smaller than range_min.  This function
     * can be used to vertically compress obstacles into a single LaserScan.
     * 
     * @param scan_height Number of pixels centered around the center of the image to compress into the LaserScan.
     * 
     */
    void set_scan_height(const int scan_height);
    
    /**
     * Sets the frame_id for the output LaserScan.
     * 
     * Output frame_id for the LaserScan.  Will probably NOT be the same frame_id as the depth image.
     * Example: For OpenNI cameras, this should be set to 'camera_depth_frame' while the camera uses 'camera_depth_optical_frame'.
     * 
     * @param output_frame_id Frame_id to use for the output sensor_msgs::LaserScan.
     * 
     */
    void set_output_frame(const std::string output_frame_id);

    /**
     * Sets the minimum, maximum height threshold for the rectified pointcloud.
     * Inside the function, we rectify depth image to 3D points in world frame. Since the points picked up from floor or high ceiling are mostly noise, we set the constraint to filter them out.
     *
     * @param height_min Minimum height for a rectified 3D point.
     * @param height_max Maximum height for a rectified 3D point.
     */
    void set_height_limits(const float height_min, const float height_max);

  private:

    /**
     * Computes euclidean length of a cv::Point3d (as a ray from origin)
     * 
     * This function computes the length of a cv::Point3d assumed to be a vector starting at the origin (0,0,0).
     * 
     * @param ray The ray for which the magnitude is desired.
     * @return Returns the magnitude of the ray.
     * 
     */
    double magnitude_of_ray(const cv::Point3d& ray) const;

    /**
     * Computes the angle between two cv::Point3d
     * 
     * Computes the angle of two cv::Point3d assumed to be vectors starting at the origin (0,0,0).
     * Uses the following equation: angle = arccos(a*b/(|a||b|)) where a = ray1 and b = ray2.
     * 
     * @param ray1 The first ray
     * @param ray2 The second ray
     * @return The angle between the two rays (in radians)
     * 
     */
    double angle_between_rays(const cv::Point3d& ray1, const cv::Point3d& ray2) const;
    
    /**
     * Determines whether or not new_value should replace old_value in the LaserScan.
     * 
     * Uses the values of range_min, and range_max to determine if new_value is a valid point.  Then it determines if
     * new_value is 'more ideal' (currently shorter range) than old_value.
     * 
     * @param new_value The current calculated range.
     * @param old_value The current range in the output LaserScan.
     * @param range_min The minimum acceptable range for the output LaserScan.
     * @param range_max The maximum acceptable range for the output LaserScan.
     * @return If true, insert new_value into the output LaserScan.
     * 
     */
    bool use_point(const float new_value, const float old_value, const float range_min, const float range_max) const;


      // Polynomial approximating arctangenet on the range -1,1.
      // Max error < 0.005 (or 0.29 degrees)
      // Implementation from: https://www.dsprelated.com/showarticle/1052.php
      double ApproxAtan(double z) const
      {
        const float n1 = 0.97239411f;
        const float n2 = -0.19194795f;
        return (n1 + n2 * z * z) * z;
      }

      double ApproxAtan2(double y, double x) const
      {

        if (x != 0.0f)
        {
          if (fabsf(x) > fabsf(y))
          {
            const double z = y / x;
            if (x > 0.0)
            {
              // atan2(y,x) = atan(y/x) if x > 0
              return ApproxAtan(z);
            }
            else if (y >= 0.0)
            {
              // atan2(y,x) = atan(y/x) + PI if x < 0, y >= 0
              return ApproxAtan(z) + M_PI;
            }
            else
            {
              // atan2(y,x) = atan(y/x) - PI if x < 0, y < 0
              return ApproxAtan(z) - M_PI;
            }
          }
          else // Use property atan(y/x) = PI/2 - atan(x/y) if |y/x| > 1.
          {
            const double z = x / y;
            if (y > 0.0)
            {
              // atan2(y,x) = PI/2 - atan(x/y) if |y/x| > 1, y > 0
              return -ApproxAtan(z) + M_PI_2;
            }
            else
            {
              // atan2(y,x) = -PI/2 - atan(x/y) if |y/x| > 1, y < 0
              return -ApproxAtan(z) - M_PI_2;
            }
          }
        }
        else
        {
          if (y > 0.0f) // x = 0, y > 0
          {
            return M_PI_2;
          }
          else if (y < 0.0f) // x = 0, y < 0
          {
            return -M_PI_2;
          }
        }
        return 0.0f; // x,y = 0. Could return NaN instead.
      }

    /**
    * Converts the depth image to a laserscan using the DepthTraits to assist.
    * 
    * This uses a method to inverse project each pixel into a LaserScan angular increment.  This method first projects the pixel
    * forward into Cartesian coordinates, then calculates the range and angle for this point.  When multiple points coorespond to
    * a specific angular measurement, then the shortest range is used.
    * 
    * @param depth_msg The UInt16 or Float32 encoded depth message.
    * @param cam_model The image_geometry camera model for this image.
    * @param scan_msg The output LaserScan.
    * @param scan_height The number of vertical pixels to feed into each angular_measurement.
    * @param depthOpticalTransform The transform from /map to /camera_depth_optical_frame
    *
    */
    template<typename T>
    void convert(const sensor_msgs::ImageConstPtr& depth_msg, const image_geometry::PinholeCameraModel& cam_model, 
		 const sensor_msgs::LaserScanPtr& scan_msg, const int& scan_height,
		 const tf::StampedTransform& depthOpticalTransform) const{
      // Use correct principal point from calibration
      float center_x = cam_model.cx();
      float center_y = cam_model.cy();

      // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
      double unit_scaling = depthimage_to_laserscan::DepthTraits<T>::toMeters( T(1) );
      float constant_x = unit_scaling / cam_model.fx();
      float constant_y = unit_scaling / cam_model.fy();

      const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
      int row_step = depth_msg->step / sizeof(T);

      // store specific element we want to use
      double tf_basis_2_0 = depthOpticalTransform.getBasis()[2][0];
      double tf_basis_2_1 = depthOpticalTransform.getBasis()[2][1];
      double tf_basis_2_2 = depthOpticalTransform.getBasis()[2][2];
      double tf_origin_z = depthOpticalTransform.getOrigin().z();

      // determine lower bound and upper_bound of pixel row to scan
      // given height_min_, range_min_, calculate the corresponding row in the image and start depth image conversion from there
      int lower_bound = (height_min_ - tf_origin_z) / range_min_ / 1000 / constant_y + center_y;
      lower_bound = std::max((int)center_y - scan_height / 2, std::max(0, lower_bound));
      int upper_bound = (height_max_ - tf_origin_z) / range_min_ / 1000 / constant_y + center_y;
      upper_bound = std::min(lower_bound + scan_height, std::min(upper_bound, (int)depth_msg->height));

      ROS_DEBUG("Upper_bound and lower_bound of depth image scan band are set to: %d and %d", upper_bound, lower_bound);

      depth_row += lower_bound * row_step; // Offset to starting pixel

      for(int v = lower_bound; v < upper_bound; v++, depth_row += row_step){
        for (int u = 0; u < (int)depth_msg->width; u++) // Loop over each pixel in row
        {
          T depth = depth_row[u];

          double r = depth; // Assign to pass through NaNs and Infs
          double th = -atan2((double)(u - center_x) * constant_x, unit_scaling); // Atan2(x, z), but depth divides out
          int index = (th - scan_msg->angle_min) / scan_msg->angle_increment;

          if (depthimage_to_laserscan::DepthTraits<T>::valid(depth)){ // Not NaN or Inf
            // Calculate in XYZ
            double x = (u - center_x) * depth * constant_x;
            double y = (v - center_y) * depth * constant_y;
            double z = depthimage_to_laserscan::DepthTraits<T>::toMeters(depth);

            // Early return while z is out of range
            if(z > range_max_ || z < range_min_){
              continue;
            }

            double rectified_height = x * tf_basis_2_0 + y * tf_basis_2_1 + z * tf_basis_2_2 + tf_origin_z;

            if( rectified_height < height_min_|| rectified_height > height_max_){
              continue;
            }

            // Calculate actual distance
            r = sqrt(pow(x, 2.0) + pow(z, 2.0));

            // Determine if this point should be used.
            if(use_point(r, scan_msg->ranges[index], scan_msg->range_min, scan_msg->range_max)){
              scan_msg->ranges[index] = r;
            }
          }
        }
      }
    }
    
    image_geometry::PinholeCameraModel cam_model_; ///< image_geometry helper class for managing sensor_msgs/CameraInfo messages.
    
    float scan_time_; ///< Stores the time between scans.
    float range_min_; ///< Stores the current minimum range to use.
    float range_max_; ///< Stores the current maximum range to use.
    int scan_height_; ///< Number of pixel rows to use when producing a laserscan from an area.
    std::string output_frame_id_; ///< Output frame_id for each laserscan.  This is likely NOT the camera's frame_id.

    float height_min_; ///< height threshold for rectified laser point
    float height_max_;

    ros::Time firstOpticalFrameTime_;
  };
  
  
}; // depthimage_to_laserscan

#endif
