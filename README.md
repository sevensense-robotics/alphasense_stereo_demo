# Stereo demo for Alphasense Core


The front camera pair of the Alphasense Core Development Kit can be used to
generate disparity maps and point clouds for several robotics applications. To
allow everyone to benefit from such potential, we make
available a minimal ROS based `stereo_image_proc` pipeline. Please note that some basic Linux/Ubuntu
and ROS knowledge is required to set it up.

*NOTE: This application is based on an OpenCV implementation for stereo-vision and provided as an example for interfacing to Alphasense Core’s driver APIs. Therefore, we recommend to use it only for demo purposes.*

![gif](/doc/images/alphasense.gif)

## Getting started with Alphasense Core

Please follow the
[Getting started instructions](https://github.com/sevensense-robotics/alphasense_core_manual/blob/master/pages/getting_started.md)
from the
[Alphasense Core manual](https://github.com/sevensense-robotics/alphasense_core_manual)
to get your Alphasense Core Development Kit up and running with the ROS driver.


## Building the example

The following instructions are tested on Ubuntu 18.04 with ROS Melodic, a
[ROS installation](http://wiki.ros.org/melodic/Installation/Ubuntu) is
therefore required.
We recommend to use the `ros-melodic-desktop` installation in order to get
access to the `rqt` and `rviz` visualization tools.

On top of that, the following libraries have to be installed:

```
sudo apt install python-catkin-tools ros-melodic-stereo-image-proc \
        ros-melodic-image-transport ros-melodic-cv-bridge
```

After that, create a new catkin workspace and clone the necessary dependencies
(`catkin_simple`, `opencv3_catkin` and `yaml_cpp_catkin`):

```
export CATKIN_WS=~/alphasense_ws
mkdir -p $CATKIN_WS/src
cd $CATKIN_WS
catkin init
catkin config --extend /opt/ros/melodic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
cd $CATKIN_WS/src
git clone https://github.com/catkin/catkin_simple.git
git clone https://github.com/ethz-asl/opencv3_catkin.git
git clone https://github.com/ethz-asl/yaml_cpp_catkin
git clone https://github.com/sevensense-robotics/alphasense_stereo_demo.git
```

The build can be triggered with the following command:

```
cd $CATKIN_WS
catkin build alphasense_stereo
```

Note that building the code will clone the source code of `opencv` and
`yaml_cpp` during the first build. Depending on the speed of your internet
connection and your CPU, this step can take a while.

In order to use the code, source the catkin workspace:

```
source $CATKIN_WS/devel/setup.bash
```

## Running stereo on your Development Kit

First [launch the Alphasense ROS driver](https://github.com/sevensense-robotics/alphasense_core_manual/blob/master/pages/getting_started.md#2-launching-the-ros-driver).

Run the following command to start the stereo matching:

```
roslaunch alphasense_stereo alphasense_stereo.launch calibration_file:=/path/to/calibration_file.yaml
```

Note that `/path/to/calibration_file.yaml` must be adjusted to match the actual
path of the factory calibration which was provided together with the sensor.
For this usage example, we are using the [Kalibr](https://github.com/ethz-asl/kalibr)
file format.

To visualize the point cloud, run the following command in a different terminal:

```
rviz -d ~/alphasense_ws/src/alphasense_stereo_demo/alphasense_stereo/rviz/stereo.rviz
```

## FAQ

### 1. Why don't you just use the ROS native [stereo_image_proc](http://wiki.ros.org/stereo_image_proc) directly?

ROS only supports the pinhole model combined with the widely used rad-tan
distortion model natively. Our camera has a wide field of view and the pinhole
model fails to describe the strong distortion here. Therefore, we are using the
[fisheye](https://docs.opencv.org/3.4/db/d58/group__calib3d__fisheye.html)
model from OpenCV in this example for the rectification. This is a perspective
projection model combined with an equidistant distortion model. We added the
OpenCV based fisheye undistortion and rectification functionalities, and
constructed a `camera_info` topic conforming to ROS standards. Except for the
image undistortion we use the native `stereo_image_proc` ROS components (block
matching, point projection).

### 2. My point cloud looks bad. Is there something wrong with the sensor?

We would like to emphasize that this is only a demo application that uses
off-the-shelf OpenCV and ROS components to create a dense stereo image. Having
said that, if you notice a sub-optimal quality of the point cloud, we recommend
the following steps:
1. First, make sure that you are using the correct factory calibration file we
   provided together with the sensor. Damages to the frame of the Development
   Kit will invalidate the factory calibration, and the same will happen when
   disassembling the frame, therefore we strongly advise against doing that.
2. Depending on your application, you might want to tune the parameters a bit.
   Set the parameter `enable_rqt_reconfigure` to true in the
   `alphasense_stereo.launch` file in order to tune the parameters while the
   block matching is running. Be aware that stereo block matching DOES NOT work
   when there is no texture on the surface, or when the environment is dark.
