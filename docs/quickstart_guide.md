# ISMR 2024 demo quickstart guide

## Setup 

Install [usb-cam package](https://github.com/ros-drivers/usb_cam?tab=readme-ov-file)
```bash
sudo apt-get install ros-noetic-usb-cam
```

Then in your catkin workspace add the following packages

Clone `ar_tag_alvar` package
```bash
git clone https://github.com/ros-perception/ar_track_alvar
cd ar_track_alvar
git checkout noetic-devel
```

Clone `ar_tag_tool_box` package
```bash
git clone https://github.com/atomoclast/ar_tag_toolbox
```

Finally compile your catkin workspace with 
```bash
catkin build
```

## Camera calibration

Next step is to calibratate your USB camera using the ros `camera_calibration` package.

TODO.

## Running the system

1. Run the usb_cam package
```bash
roslaunch ar_tag_toolbox usb_cam.launch
roslaunch ar_tag_toolbox ar_track_usb_cam.launch
```

2. Run node to overlay coordinate frames in image stream.
```bash
rosrun ismr2024_demo_needle_grasping show_poses_in_board.py
```
