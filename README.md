[![](https://github.com/ros-drivers/velodyne/workflows/Basic%20Build%20Workflow/badge.svg)](https://github.com/ros-drivers/velodyne/actions)

About this fork
===============

I created this fork of the ROS velodyne library to make it lighter that allows me to use only one node for both reading and conversion to PointCloud2. 
For my purpose, I won't need the part of the library that does the laser scan so I'll remove that part.
I would also like to add some new configuration parameters and fix some bugs that I encountered using it.

**Warning: This project is still in development.**

**NEW EDIT:**
Finally the code is working but unfortunately now it is only compatible with VLP-16 models, I hope in the future to be able to modify the code to make it work with all possible lidars (it is not a very difficult thing, it only takes time and study of the documentation), for now I intend to focus on other points of the node, such as its performance.

The changes made to the original repo in particular are:
- removal of the nodelet (now it is a single node).
- removal of the dynamic configurator.
- removal of the disordered cloud and modification of the ordered one.
- adding some parameters (FOV orizzontale).

About the main velodyne project
========

Velodyne<sup>1</sup> is a collection of ROS<sup>2</sup> packages supporting `Velodyne high
definition 3D LIDARs`<sup>3</sup>.

**Warning**:

  The master branch normally contains code being tested for the next
  ROS release.  It will not always work with every previous release.
  To check out the source for the most recent release, check out the
  tag `<version>` with the highest version number.

The current ``master`` branch works with ROS Kinetic and Melodic.
CI builds are currently run for Kinetic and Melodic.

- <sup>1</sup>Velodyne: http://www.ros.org/wiki/velodyne
- <sup>2</sup>ROS: http://www.ros.org
- <sup>3</sup>`Velodyne high definition 3D LIDARs`: http://www.velodynelidar.com/lidar/lidar.aspx
