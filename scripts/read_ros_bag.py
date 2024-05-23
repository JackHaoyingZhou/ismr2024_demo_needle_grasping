import os
import sys
import time
import rospy
import rosbag
import numpy as np
from glob import glob
from scipy.spatial.transform import Rotation as R
from utility import load_rosbag_list, convert_msg_to_frame, convert_frame_to_mat, read_marker_msg
from typing import List
dynamic_path = os.path.abspath(__file__ + "/../../")
# print(dynamic_path)
sys.path.append(dynamic_path)
from geometry_msgs.msg import PoseStamped
from aruco_detection.msg import MarkerPose, MarkerPoseArray
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib
matplotlib.use('Qt5Agg')
# matplotlib.use('TkAgg')
from PyKDL import Frame

def draw_axes(ax:plt.axes, frame:Frame, arrow_length:float=0.05)->None:
    '''
    Plot the axes lines
    :param ax: plot figure axis
    :param frame: the pose of the coordinate system
    :param arrow_length: length of the arrows, default to be 0.1
    :return:
    '''
    # Plotting the axes lines
    # Draw lines from the origin to the positive ends of each axis
    ax.quiver(frame.p.x(), frame.p.y(), frame.p.z(), arrow_length*frame.M.UnitX()[0], arrow_length*frame.M.UnitX()[1],
              arrow_length*frame.M.UnitX()[2], color='r', arrow_length_ratio=0.1)  # x-axis
    ax.quiver(frame.p.x(), frame.p.y(), frame.p.z(), arrow_length*frame.M.UnitY()[0], arrow_length*frame.M.UnitY()[1],
              arrow_length*frame.M.UnitY()[2], color='b', arrow_length_ratio=0.1)  # y-axis
    ax.quiver(frame.p.x(), frame.p.y(), frame.p.z(), arrow_length*frame.M.UnitZ()[0], arrow_length*frame.M.UnitZ()[1],
              arrow_length*frame.M.UnitZ()[2], color='g', arrow_length_ratio=0.1)  # z-axis

if __name__ == '__main__':
    bag_folder = os.path.join(dynamic_path, "test_data")
    bag_file_list = load_rosbag_list(bag_folder)
    rosbag_name = bag_file_list[0]
    # PlotFig = False
    PlotFig = True

    print('The ROS bag name is: ', rosbag_name)

    bag = rosbag.Bag(rosbag_name)
    topics = list(bag.get_type_and_topic_info()[1].keys())
    types = [val[0] for val in bag.get_type_and_topic_info()[1].values()]

    msg_psm2 = []
    msg_psm2_jaw = []
    msg_marker = []

    # draw_axes()

    psm2_frame = []
    psm2_mtx = []
    psm2_jaw = []
    aruco_markers = []

    ### Read PSM2
    for _, msg, t in bag.read_messages(topics='/PSM2/setpoint_cp'):
        psm2_frame_temp = convert_msg_to_frame(msg)
        psm2_mtx_temp = convert_frame_to_mat(psm2_frame_temp)
        psm2_frame.append(psm2_frame_temp)
        psm2_mtx.append(psm2_mtx_temp)

    ### Read PSM2 Jaw
    for _, msg, t in bag.read_messages(topics='/PSM2/jaw/setpoint_js'):
        psm2_jaw_temp = msg.position[0]
        psm2_jaw.append(psm2_jaw_temp)

    ### Read AR tags
    for _, msg, t in bag.read_messages(topics='/aruco/marker_poses'):
        marker_temp = read_marker_msg(msg)
        # # if you want to check all keys in the dict
        # key_list = list(marker_temp.keys())
        aruco_markers.append(marker_temp)

    diff_tags = []
    tag_all_detect = []

    for marker_dict in aruco_markers:
        if len(marker_dict) == 2:
            ar_tag0_temp = marker_dict[0]
            ar_tag1_temp = marker_dict[1]
            diff_tag_temp = ar_tag0_temp.Inverse() * ar_tag1_temp
            diff_tags.append(diff_tag_temp)
            tag_all_detect.append(marker_dict)

    select_idx = int((len(tag_all_detect)) / 2)

    ar_tag0_pose = tag_all_detect[select_idx][0]
    ar_tag1_pose = tag_all_detect[select_idx][1]

    if PlotFig:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        ax.set_xlim([-0.3, 0.3])
        ax.set_ylim([-0.3, 0.3])
        ax.set_zlim([-0.3, 0.3])

        # for frame in psm2_frame:
        #     draw_axes(ax, frame)
        draw_axes(ax, psm2_frame[-1])
        draw_axes(ax, ar_tag0_pose)
        # draw_axes(ax, ar_tag1_pose)

    plt.show()

    # from the plot figure, we can see that:
    # psm frame: x, y, z
    # marker frame: y, x, z
    # AKA, rotate 90 deg ccw along z axis