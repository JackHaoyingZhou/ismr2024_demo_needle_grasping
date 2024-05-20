import os
import sys
import rospy
import rosbag
import numpy as np
from glob import glob
from scipy.spatial.transform import Rotation as R
from typing import List
dynamic_path = os.path.abspath(__file__ + "/../../")
# print(dynamic_path)
sys.path.append(dynamic_path)
from geometry_msgs.msg import PoseStamped
from aruco_detection.msg import MarkerPose, MarkerPoseArray

def load_rosbag_list(bag_folder:str)->List[str]:
    '''
    Load all ROS bag files in a folder.
    :param bag_folder: the path of the selected folder
    :return: sorted file list
    '''
    bag_file_raw_list = glob(os.path.join(bag_folder, "*.bag"))
    bag_file_list = sorted(bag_file_raw_list, key=lambda x: x.split('/')[-1].split('.')[0].split('_')[-1])
    return bag_file_list


if __name__ == '__main__':
    bag_folder = os.path.join(dynamic_path, "test_data")
    bag_file_list = load_rosbag_list(bag_folder)
    rosbag_name = bag_file_list[0]

    bag = rosbag.Bag(rosbag_name)
    topics = list(bag.get_type_and_topic_info()[1].keys())
    types = [val[0] for val in bag.get_type_and_topic_info()[1].values()]

    msg_psm2 = []
    msg_psm2_jaw = []
    msg_marker = []

    for _, msg, t in bag.read_messages(topics='/PSM2/setpoint_cp'):
        msg_psm2.append(msg)

    for _, msg, t in bag.read_messages(topics='/PSM2/jaw/setpoint_js'):
        msg_psm2_jaw.append(msg)

    for _, msg, t in bag.read_messages(topics='/aruco/marker_poses'):
        msg_marker.append(msg)

