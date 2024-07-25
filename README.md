depthimage_to_laserscan
=======================

A ROS 2 driver to convert a depth image to a laser scan for use with navigation and localization.

Published Topics
----------------
* `scan` (`sensor_msgs/msg/LaserScan`) - The laser scan computed from the depth image.

Subscribed Topics
-----------------
* `depth_camera_info` (`sensor_msgs/msg/CameraInfo`) - The camera info.
* `depth` (`sensor_msgs/msg/Image`) - The depth image.

Parameters
----------
* `scan_time` (float) - The time in seconds between scans to report to the consumer of the LaserScan message.  This is set directly in the published message.  Defaults to 0.033 seconds.
* `range_min` (float) - The minimum distance in meters a projected point should be.  Points closer than this are discarded.  Defaults to 0.45 meters.
* `range_max` (float) - The maximum distance in meters a projected point should be.  Points further than this are discarded.  Defaults to 10.0 meters.
* `scan_height` (int) - The number of rows from the depth image to use for the laser projection.  Defaults to 1.
* `scan_offset` (float) - The height ratio of the depth image defining which height the depth data should be taken from (0.0=top row of image, 1.0=bottom row). Defaults to 0.5. 
* `output_frame` (string) - The frame id to publish in the LaserScan message.  Defaults to "camera_depth_frame".
