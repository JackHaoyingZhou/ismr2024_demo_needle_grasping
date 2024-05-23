import os
import sys
import numpy as np
from PyKDL import Frame, Rotation, Vector
from typing import Union, List
from geometry_msgs.msg import PoseStamped
from aruco_detection.msg import MarkerPoseArray
from glob import glob
dynamic_path = os.path.abspath(__file__ + "/../../")
# print(dynamic_path)
sys.path.append(dynamic_path)

def cartesian_interpolate_step(T_curr: Frame, T_goal:Frame, max_delta:float=0.01, deadband:float=0.01)->Union[Frame, bool]:
    '''
    Cartesian interpolation step for moving from current pose to goal pose
    :param T_curr: Current frame
    :param T_goal: Goal frame
    :param max_delta: scaling factor for max distance between current and goal pose
    :param deadband: tolerance for distance between current and goal pose when it considers to be done
    :return: Cartesian interpolation step, a flag whether it is done or not
    '''
    error = np.zeros(6)
    pe = T_goal.p - T_curr.p
    re = (T_curr.M.Inverse() * T_goal.M).GetRPY()
    for i in range(6):
        if i < 3:
            error[i] = pe[i]
        else:
            error[i] = re[i-3]

    done = False
    error_max = max(np.abs(error))
    if error_max <= deadband:
        error_scaled = error * 0.
        done = True
    else:
        error_scaled = error / error_max

    error_scaled = error_scaled * max_delta

    T_step = Frame(Rotation.RPY(error_scaled[3], error_scaled[4], error_scaled[5]),
                   Vector(error_scaled[0], error_scaled[1], error_scaled[2]))
    return T_step, done

def convert_msg_to_frame(msg:PoseStamped)->Frame:
    '''
    Converts PoseStamped message to PyKDL Frame
    :param msg: input msgs
    :return: Frame converted from PoseStamped
    '''
    pose_info = msg.pose
    frame = Frame()
    frame.p = Vector(pose_info.position.x, pose_info.position.y, pose_info.position.z)
    frame.M = Rotation.Quaternion(pose_info.orientation.x, pose_info.orientation.y, pose_info.orientation.z,
                                  pose_info.orientation.w)
    return frame

def convert_frame_to_mat(frame:Frame)->np.ndarray:
    '''
    Converts PyKDL Frame to numpy matrix
    :param frame: the frame to convert
    :return: the converted 4x4 transformation matrix
    '''
    np_mtx = np.eye(4)
    for i in range(3):
        for j in range(3):
            np_mtx[i, j] = frame.M[i, j]

    for i in range(3):
        np_mtx[i, 3] = frame.p[i]
    return np_mtx

def convert_mat_to_frame(mtx:np.ndarray)->Frame:
    '''
    Converts numpy matrix to Frame
    :param mtx: the np matrix to convert
    :return: the converted frame
    '''
    frame = Frame(Rotation.RPY(0, 0, 0), Vector(0, 0, 0))
    for i in range(3):
        for j in range(3):
            frame.M[i, j] = mtx[i, j]
    for i in range(3):
        frame.p[i] = mtx[i, 3]
    return frame

def read_marker_msg(msg:MarkerPoseArray)->dict:
    '''
    Read marker messages
    :param msg: the msg to convert
    :return: a dictionary with all markers' information
    '''
    marker_info = msg.markers
    out_dict = dict()
    for marker in marker_info:
        ## marker's type is MarkerPose [id: int, pose:PoseStamp]
        label = marker.id
        pose = convert_msg_to_frame(marker)
        out_dict[label] = pose
    return out_dict

def load_rosbag_list(bag_folder:str)->List[str]:
    '''
    Load all ROS bag files in a folder.
    :param bag_folder: the path of the selected folder
    :return: sorted file list
    '''
    bag_file_raw_list = glob(os.path.join(bag_folder, "*.bag"))
    bag_file_list = sorted(bag_file_raw_list, key=lambda x: x.split('/')[-1].split('.')[0].split('_')[-1])
    return bag_file_list

if __name__=="__main__":
    test = 1