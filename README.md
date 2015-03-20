pcl_auto_seg
============

ROS Indigo package for segmenting a block from an RGBD scene and publishing the transform from the block to the camera.

Description
===========

The primary purpose of this package is to provide the ROS node seg_block which looks for a block in an RGBD scene and publishes the transform from the block to the camera.  The node that does this work is seg_block and is basically a modification of the PCl tutorial [Aligning Object Templates to a Point Cloud](http://pointclouds.org/documentation/tutorials/template_alignment.php).  It works by creating several cube models of varying sizes and finding the one that works the best.

Several assumptions are made in order for this node to work.  One assumption is that the block is sitting some surface flat enough and large enough that it can be recognized as a plane and removed from the image.  Another assumption is that the cube is 3 to 5.5 cm on a side.  It also assumes that the block is no more than 1.5 m away from the camera.

The package also contains two utility nodes:  save_pt_cloud and view_pt_cloud.  The save_pt_cloud node saves the first RGBD image it receives to disk and exits.  The view_pt_cloud node loads a saved point cloud from disk and displays it in a visualization window.

The seg_block Node
==================

The seg_block node operates by subscribing to the camera/depth_registered/points topic and running the recognition algorithm on each scene received.  If a block has been found the node will publish a geometry_msgs::Pose object over the topic /block_pose.  

Be aware that this node can be prone to false positives.  Sometimes the block templates will be matched up with corners of a wall or something else.  The code might require some changes to add additional filtering before running the template matching.


Required Hardware
=================

I made use of the [ASUS Xtion PRO LIVE](http://www.asus.com/us/Multimedia/Xtion_PRO_LIVE/) while developing this software, but the Kinect should also work.  The camera just needs to be compatible with the OpenNI2 drivers.


Software Dependencies
=====================

This package relies on two libraries to run:  OpenNI2 and PCL.

To install OpenNI2 run the following commands to install everything you need for using it in ROS Indigo.

```bash
sudo apt-get install ros-indigo-openni2-camera
sudo apt-get install ros-indigo-openni2-launch
```

To install PCL follow the commands on the page [Prebuild binaries for Linux](http://pointclouds.org/downloads/linux.html).

Running the Nodes
=================

To run the seg_block node launch the seg_block.launch file.

```bash
roslaunch pcl_auto_seg seg_block.launch
```

To run the view_pt_cloud node, first open view_pt_cloud.launch and modify the line

```xml
<param name="file_name" value="/home/mongeon/block_on_floor.pcd"/>
```

to set value to the file location.  Then launch the file

```bash
roslaunch pcl_auto_seg view_pt_cloud.launch
```

This node does not require any connected camera to operate since it only uses a file on disk.

To run the save_pt_cloud node run the save_pt_cloud.launch file.

```bash
roslaunch pcl_auto_seg save_pt_cloud.launch
```

