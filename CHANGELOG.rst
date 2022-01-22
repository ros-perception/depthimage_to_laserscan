^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package depthimage_to_laserscan
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.5.0 (2022-01-22)
------------------
* Install includes to ${PROJECT_NAME} folder and use modern CMake (`#58 <https://github.com/ros-perception/depthimage_to_laserscan/issues/58>`_)
* Fix the launch files to work with Rolling. (`#59 <https://github.com/ros-perception/depthimage_to_laserscan/issues/59>`_)
* Contributors: Chris Lalancette, Shane Loretz

2.3.1 (2021-04-08)
------------------
* Cleanup the CMakeLists.txt (`#51 <https://github.com/ros-perception/depthimage_to_laserscan/issues/51>`_)
* Contributors: Chris Lalancette

2.3.0 (2020-05-28)
------------------
* Make DepthImageToLaserScan more RAII.
* Fixes pointed out by clang-tidy.
* Contributors: Chris Lalancette

2.2.5 (2019-11-13)
------------------
* Update the README.md to describe the topics and parameters.
* Add in a launch file for composable node.
* Rename launch file to conform to recommendations.
* Remove unnecessary Depth.cfg.
* Make depthimage_to_laserscan composable.
* Rename header files to .hpp
* Style cleanup.
* Contributors: Chris Lalancette

2.2.4 (2019-10-23)
------------------
* Export interfaces for Win32 Shared Lib (`#44 <https://github.com/ros-perception/depthimage_to_laserscan/issues/44>`_)
* Contributors: Sean Yen

2.2.2 (2019-09-27)
------------------
* update code to be consist with new ros2 core (`#42 <https://github.com/ros-perception/depthimage_to_laserscan/issues/42>`_)
* Contributors: Gary Liu

2.2.1 (2019-03-29)
------------------
* Port depthimage_to_laserscan to ROS2.
* Import ROS2 changes from https://github.com/ros2/depthimage_to_laserscan.git to upstream
* Contributors: Chad Rockey, Chris Lalancette, Mikael Arguedas, Shane Loretz, dhood

1.0.8 (2019-02-07)
------------------
* Merge pull request `#27 <https://github.com/ros-perception/depthimage_to_laserscan/issues/27>`_ from ros-perception/mikaelarguedas-patch-1
  update to use non deprecated pluginlib macro
* update to use non deprecated pluginlib macro
* Contributors: Chad Rockey, Mikael Arguedas

1.0.7 (2014-06-16)
------------------
* Merge pull request `#11 <https://github.com/ros-perception/depthimage_to_laserscan/issues/11>`_ from ros-perception/hydro-devel
  Update indigo devel with angle fix
* Merge pull request `#8 <https://github.com/ros-perception/depthimage_to_laserscan/issues/8>`_ from vliedel/hydro-devel
  fixed angle_increment
* fixed angle_increment
* Merge pull request `#7 <https://github.com/ros-perception/depthimage_to_laserscan/issues/7>`_ from bulwahn/hydro-devel
  check for CATKIN_ENABLE_TESTING
* check for CATKIN_ENABLE_TESTING
* Added ARCHIVE DESTINATION
* Contributors: Chad Rockey, Lukas Bulwahn, vliedel

1.0.6 (2013-10-31)
------------------
* Removed y component from projected beam radius.

1.0.5 (2013-08-21)
------------------
* No more Willow Garage email.
* Merge pull request `#4 <https://github.com/ros-perception/depthimage_to_laserscan/issues/4>`_ from ericperko/groovy-devel
  Cleaned up CMakeLists a bit
* Cleaned up CMakeLists a bit

1.0.4 (2013-01-22)
------------------
* Update version to 1.0.4
* Merge pull request `#3 <https://github.com/ros-perception/depthimage_to_laserscan/issues/3>`_ from ahendrix/fix_testdeps
  Fix testdeps
* Removed redundant test-depends.
* Remove old makefile.

1.0.3 (2012-12-23)
------------------
* fix exported libraries (fix `#2 <https://github.com/ros-perception/depthimage_to_laserscan/issues/2>`_)

1.0.2 (2012-12-19)
------------------
* fix dyn reconf
* Updated description.
* Updated readme to link to wiki.

1.0.1 (2012-12-14)
------------------
* Update package.xml
  New release for missing catkin build_depend.
* Merge pull request `#1 <https://github.com/ros-perception/depthimage_to_laserscan/issues/1>`_ from ablasdel/groovy-devel
  updated catkin buildtool_depend
* updated catkin buildtool_depend

1.0.0 (2012-12-05)
------------------
* Cleanup of the CMakeLists.txt
* First attempt at Catkinizing.
* Pushing fuerte-devel to github.
* Initial commit
