# 2024 ISMR Workshop Demo Quick Start Guide

## Hardware Setup

To complete this demo, you would need the following items:

1. A [dVRK](https://research.intusurg.com/index.php/Main_Page) PSM Arm
2. A camera
3. dVRK Hand-Eye Calibration Kit (a cube with an Aruco marker attached)
4. A paper with some Aruco markers printed on it
5. An external light source
6. Some mechanical support for the system (you can have your own design)

For the camera, the demo is using a depstech borescope.

## Preparation

You may need to print some Aruco markers with certain size. You can take advantage of the [cv_marker_generator](https://github.com/JackHaoyingZhou/cv_marker_generator) 
GitHub Repository.

## Software Setup 

***Note: you can skip step 1-3 if you have already had dVRK and its video pipeline properly installed. You can replace 
`~/catkin_ws_dvrk` with your catkin workspace path***

1. create a catkin workspace

Run the following command in your terminal

```bash
cd ~
mkdir catkin_ws_dvrk
cd catkin_ws_dvrk
```

2. install the dVRK 

Please follow the [dVRK ROS 1 install instructions](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/CatkinBuild)

3. install gscam for dVRK video pipeline

***Note: This note is only for Ubuntu 20.04 OS. If you happen to have different OS, please go through the 
[video pipeline README](https://github.com/jhu-dvrk/dvrk_video/blob/devel/README.md) for more information.***

Firstly, clone the noetic-devel branch of a customized gscam repository https://github.com/JackHaoyingZhou/gscam into the `src`
folder of your dVRK workspace. And then manually compile it:

```bash
cd ~/catkin_ws_dvrk/src
git clone https://github.com/JackHaoyingZhou/gscam.git
cd gscam
git checkout noetic-devel
cd ..
catkin build --summary
```

4. Install the dependencies

clone and compile from source to install the [aruco_detection](https://github.com/JackHaoyingZhou/aruco_detection) and 
the [ISMR demo](https://github.com/JackHaoyingZhou/ismr2024_demo_needle_grasping) packages

```bash
cd ~/catkin_ws_dvrk/src
git clone https://github.com/JackHaoyingZhou/aruco_detection.git
git clone https://github.com/JackHaoyingZhou/ismr2024_demo_needle_grasping.git
catkin build --summary
```

5. Source your catkin workspace

```bash
source ~/catkin_ws_dvrk/devel/setup.bash
```

## Calibrations

1. create a catkin package 

You will need to create a new package to store the camera calibration results. At that point, choose a name for your camera. 
For the demo application, we will use `depstech`:

```bash
cd ~/catkin_ws_dvrk/src
catkin_create_pkg depstech
catkin build --summary
source ~/catkin_ws_dvrk/devel/setup.bash
```

2. start the video streaming

This is a video capture based on a video4linux compatible source:

```bash
roslaunch ismr2024_demo_needle_grasping gscam_v4l_mjpg.launch camera_name:=depstech image_width:=1920 image_height:=1080
```

3. calibrate the camera

The calibration is performed using the ROS provided application, please refer to their documentation for the 
parameters [ROS camera calibration](https://wiki.ros.org/camera_calibration)

For the camera calibration, we are using a 12x10 calibration grid and the size of each square is 6mm:

```bash
rosrun camera_calibration cameracalibrator.py --size 12x10 --square 0.006 image:=/depstech/image_raw camera:=/depstech
```

Make sure you hit `calibration` button then hit `commit` button.

4. restart the video using the calibration

```bash
roslaunch ismr2024_demo_needle_grasping gscam_v4l_mjpg.launch camera_name:=depstech image_width:=1920 image_height:=1080 mono_proc:=True
```

5. hand-eye calibration

The calibration cube has an Aruco marker attached to it, the marker's size is 1.5mm.

**calibration script**

```bash
rosrun dvrk_camera_registration camera_registration.py -p PSM2 -m 0.015 -c /depstech
```

`-p` is used to indicate which PSM to use, `-m` represents the size of the marker, and `-c` is the namespace for your camera.

After the script starts, you need to manually move the PSM all over your desired configuration space for best hand-eye 
calibration results.

After collecting the data, the scripts will generate a couple of `.json` files with the transformation between the PSM 
and the camera. The file with `-open-cv` is used for the validation script below.

**validation script**

```bash
rosrun dvrk_camera_registration vis_gripper_pose.py -p PSM2 -c /depstech -H PSM2-registration-open-cv.json
```

If your validation results are acceptable, you need to do the following two steps:

1. **Efit your dVRK configuration files**

The script also generates a `PSM<x>_registration-dVRK.json` file that contains a transformation which you can copy-paste 
in your dVRK `console-PSM<x>.json` to define the PSM `base-frame`.

2. **Copy the `.json` files to `<ISMR demo>/config` folder**

```bash
cp ./PSM2-registration-dVRK.json ~/catkin_ws_dvrk/src/ismr2024_demo_needle_grasping/config/
cp ./PSM2-registration-open-cv.json ~/catkin_ws_dvrk/src/ismr2024_demo_needle_grasping/config/
```

## How to Run

Firstly, you need to start the video:

```bash
roslaunch ismr2024_demo_needle_grasping gscam_v4l_mjpg.launch camera_name:=depstech image_width:=1920 image_height:=1080 mono_proc:=True
```

Secondly, you need to run the Aruco detection script to enable the tracking of all visible Aruco markers:

```bash
rosrun aruco_detection aruco_detector.py
```

Last but not least, you can run the needle grasping script using default settings:

```bash
rosrun ismr2024_demo_needle_grasping grasp_needle.py
```

If you would like to visualize the frames, please run the following command in a separate terminal:

```bash
rosrun ismr2024_demo_needle_grasping show_all_frames.py
```

Alternatively, you can set your own parameters based on the [developer guid](../docs/script_devel_guide.md) for the above two running scripts. To do that, you need to run:

```bash
cd ~/catkin_ws_dvrk/src/ismr2024_demo_needle_grasping/scripts
python grasp_needle.py <your settings>
```

and 

```bash
cd ~/catkin_ws_dvrk/src/ismr2024_demo_needle_grasping/scripts
python show_all_frames.py <your settings>
```