#!/usr/bin/env python3
import os
import sys
import json
import crtk
import os
import sys
import time
import signal
import numpy as np
import rospy
import rosbag
import numpy
import PyKDL
import argparse

dynamic_path = os.path.abspath(__file__ + "/../../")
# print(dynamic_path)
sys.path.append(dynamic_path)


# simplified arm class to replay motion, better performance than
# dvrk.psm since we're only subscribing to topics we need
class arm_custom:
    # simplified jaw class to close gripper
    class __jaw_device:
        def __init__(self, ral, expected_interval, operating_state_instance):
            self.__crtk_utils = crtk.utils(self, ral, expected_interval, operating_state_instance)
            self.__crtk_utils.add_move_jp()
            self.__crtk_utils.add_servo_jp()

    def __init__(self, ral, device_namespace, expected_interval):
        # ROS initialization
        if not rospy.get_node_uri():
            rospy.init_node('simplified_arm_class', anonymous=False, log_level=rospy.WARN)
        # populate this class with all the ROS topics we need
        self.__ral = ral.create_child(device_namespace)
        self.crtk_utils = crtk.utils(self, self.__ral, expected_interval)
        self.crtk_utils.add_operating_state()
        self.crtk_utils.add_servo_jp()
        self.crtk_utils.add_move_jp()
        self.crtk_utils.add_servo_cp()
        self.crtk_utils.add_move_cp()
        jaw_ral = self.ral().create_child('/jaw')
        self.jaw = self.__jaw_device(jaw_ral, expected_interval, operating_state_instance=self)

    def ral(self):
        return self.__ral

    def check_connections(self, timeout=5.0):
        self.__ral.check_connections(timeout)

# if sys.version_info.major < 3:
#     input = raw_input

# ---------------------------------------------
# ros setup
# ---------------------------------------------


# strip ros arguments
argv = crtk.ral.parse_argv(sys.argv)

# ---------------------------------------------
# parse arguments
# ---------------------------------------------
parser = argparse.ArgumentParser()
parser.add_argument('-a', '--arm', type=str, default='PSM2',
                    choices=['PSM1', 'PSM2', 'PSM3'],
                    help='arm name corresponding to ROS topics without namespace.')
parser.add_argument('-i', '--interval', type = float, default = 0.01,
                    help = 'expected interval in seconds between messages sent by the device')
parser.add_argument('-f', '--file_json', type=str, default='three_wpi_traj.json',
                    help='json file containing the trajectory to replay.')
parser.add_argument('-j', '--jaw', action = 'store_true',
                    help = 'specify if the PSM jaw is enabled')


args = parser.parse_args(argv[1:])  # skip argv[0], script name

# ros init node
ral = crtk.ral('dvrk_needle_grasp')
has_jaw = args.jaw

# create data folder
# if not os.path.exists(os.path.join(os.getcwd(), 'data')):
#     os.mkdir(os.path.join(os.getcwd(), 'data'))
#

# ---------------------------------------------
# read commanded joint position
# ---------------------------------------------
#### Need to fix
poses = []

f = open(args.file_json)
data = json.load(f)
poses_origin = data["js"]
poses = []
jaw_poses = []

for pose_old in poses_origin:
    pose_input_temp = pose_old[:-2]
    pose_last_temp = (pose_old[-2] + pose_old[-1]) / 2.
    pose_input_temp.append(pose_last_temp)
    pose_jaw_temp = - pose_old[-2] + pose_old[-1]
    poses.append(np.array(pose_input_temp))
    jaw_poses.append(np.array([pose_jaw_temp]))

f.close()
####

# ---------------------------------------------
# prepare psm
# ---------------------------------------------
print('-- This script will replay a trajectory defined in %s on arm %s' % (args.file_json, args.arm))

# create arm
arm = arm_custom(ral, device_namespace=args.arm, expected_interval=args.interval)
# arm = arm_custom(ral.create_child(args.arm), expected_interval=args.interval)

arm.ral().check_connections()

# make sure the arm is powered
print('-- Enabling arm')
if not arm.enable(10):
    sys.exit('-- Failed to enable within 10 seconds')

print('-- Homing arm')
if not arm.home(10):
    sys.exit('-- Failed to home within 10 seconds')

input(
    '---> Make sure the arm is ready to move using joint positions\n     '
    'You need to have a tool/instrument in place and properly engaged\n     '
    'Press "Enter" when the arm is ready')

# close gripper
input('---> Press \"Enter\" to close the instrument\'s jaws')

##################################################
# close the jaw firstly
if has_jaw:
    jaw_jp = np.array([-20.0 * np.pi / 180.0])
    arm.jaw.move_jp(jaw_jp).wait()

# go to initial position and wait
input('---> Press \"Enter\" to move to start position')
if has_jaw:
    jaw_jp = jaw_poses[0]
    arm.jaw.move_jp(jaw_jp).wait()
# jaw_jp = np.array([0])
jp = poses[0]
arm.move_jp(jp).wait()


# ---------------------------------------------
# start playing trajectory and data collection
# ---------------------------------------------
# play trajectory
input('---> Press \"Enter\" to replay the recorded trajectory and collect data')

# run shell script

# main play process
counter = 0
total = len(poses)
start_time = time.time()

# for pose in poses:
for i in range(len(poses)):
    start_t = time.time()
    if has_jaw:
        jaw_jp = jaw_poses[i]
        arm.jaw.servo_jp(jaw_jp)
    # time.sleep(0.05)
    arm.servo_jp(poses[i])
    counter = counter + 1
    # report progress
    sys.stdout.write('\r-- Progress %02.1f%%' % (float(counter) / float(total) * 100.0))
    sys.stdout.flush()
    end_t = time.time()
    delta_t = args.interval - (end_t - start_t)  ###############################check console rate
    # if process takes time larger than console rate, don't sleep
    if delta_t > 0:
        time.sleep(delta_t)

print('\n--> Time to grasp the needle: %f seconds' % (time.time() - start_time))
print('--> Done!')

