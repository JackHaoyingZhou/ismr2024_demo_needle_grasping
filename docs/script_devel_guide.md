# 2024 ISMR Workshop Demo Script Guide for Developers

## Aruco Marker Size for Detection

Unfortunately, we currently hard-code the marker size in the [aruco detection code](https://github.com/JackHaoyingZhou/aruco_detection/blob/main/scripts/aruco_detector.py#L14).

If you would like to modify the marker size, you need to change the parameter `marker_length` in the mentioned code above, 
the unit of the length is meter.

## Visualization Script - [show_all_frames.py](../scripts/show_all_frames.py)

1. arguments

| argument | argument meaning                                               | default value                         |
|----------|----------------------------------------------------------------|---------------------------------------|
| -a       | PSM name corresponding to ROS topics                           | PSM2                                  |
| -f       | hand-eye calibration JSON file using Open CV coordinate system | config/PSM2-registration-open-cv.json |
| -c       | ROS namespace for the camera                                   | /depstech                             |

2. modify the default values:

You can set your own default values at line [#306-#333](https://github.com/JackHaoyingZhou/ismr2024_demo_needle_grasping/blob/main/scripts/show_all_frames.py#L311)

## Needle Grasping Script - [grasp_needle.py](../scripts/grasp_needle.py)

1. arguments

| argument | argument meaning                     | default value |
|----------|--------------------------------------|---------------|
| -a       | PSM name corresponding to ROS topics | PSM2          |
| -t       | Target Aruco Marker Id               | 1             |
| -c       | ROS namespace for the camera         | /depstech     |

2. modify the default values:

You can set your own default values at line [#287-#309](https://github.com/JackHaoyingZhou/ismr2024_demo_needle_grasping/blob/main/scripts/grasp_needle.py#L290)

3. hard-coded transformations

(1) desired position transformation offset with respect to the marker, unit: m or rad

line [#240](https://github.com/JackHaoyingZhou/ismr2024_demo_needle_grasping/blob/main/scripts/grasp_needle.py#L240)

(2) transformation offset when grasping the needle, unit: m or rad

line [#344](https://github.com/JackHaoyingZhou/ismr2024_demo_needle_grasping/blob/main/scripts/grasp_needle.py#L344)