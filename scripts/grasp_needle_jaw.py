from typing import List
import rospy
from dataclasses import dataclass, field
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped
import time
import argparse
import crtk
import cv2
import sys
from cv_bridge import CvBridge
import numpy as np
import json
from aruco_detection.msg import MarkerPose, MarkerPoseArray
import tf_conversions.posemath as pm
import os
from utility import convert_mat_to_frame, convert_frame_to_mat, convert_msg_to_frame, cartesian_interpolate_step
from PyKDL import Frame, Rotation, Vector
dynamic_path = os.path.abspath(__file__ + "/../../")
# print(dynamic_path)
sys.path.append(dynamic_path)
import copy


@dataclass
class PoseAnnotatorPSM:
    camera_matrix: np.ndarray
    dist_coeffs: np.ndarray = field(
        default_factory=lambda: np.array([0.0, 0.0, 0.0, 0.0, 0.0]).reshape((-1, 1)),
        init=False,
    )

    def __post_init__(self):
        # print(self.dist_coeffs.shape)
        pass

    def draw_pose_on_img(self, img: np.ndarray, measured_cp: np.ndarray):
        offset = np.eye(4)
        # offset[0, 3] = -0.008 # z dir
        # offset[1, 3] =  0.005 # y dir
        # offset[2, 3] = 0.008 # x dir
        pose = measured_cp @ offset

        tvec = pose[:3, 3]
        rvec = cv2.Rodrigues(pose[:3, :3])[0]

        points_3d = np.array([[[0, 0, 0]]], np.float32)
        points_2d, _ = cv2.projectPoints(
            points_3d, rvec, tvec, self.camera_matrix, self.dist_coeffs
        )

        points_2d = tuple(points_2d.astype(np.int32)[0, 0])

        img = cv2.circle(img, points_2d, 10, (0, 0, 255), -1)
        img = self.draw_axis(img, self.camera_matrix, self.dist_coeffs, pose, size=0.01)

        return img

    def draw_axis(
        self,
        img: np.ndarray,
        mtx: np.ndarray,
        dist: np.ndarray,
        pose: np.ndarray,
        size: int = 10,
    ):

        s = size
        thickness = 2
        R, t = pose[:3, :3], pose[:3, 3]
        K = mtx

        rotV, _ = cv2.Rodrigues(R)
        points = np.float32([[s, 0, 0], [0, s, 0], [0, 0, s], [0, 0, 0]]).reshape(-1, 3)
        axisPoints, _ = cv2.projectPoints(points, rotV, t, K, dist)
        axisPoints = axisPoints.astype(int)

        img = cv2.line(
            img,
            tuple(axisPoints[3].ravel()),
            tuple(axisPoints[0].ravel()),
            (255, 0, 0),
            thickness,
        )
        img = cv2.line(
            img,
            tuple(axisPoints[3].ravel()),
            tuple(axisPoints[1].ravel()),
            (0, 255, 0),
            thickness,
        )

        img = cv2.line(
            img,
            tuple(axisPoints[3].ravel()),
            tuple(axisPoints[2].ravel()),
            (0, 0, 255),
            thickness,
        )
        return img


@dataclass
class PoseAnnotatorAruco:
    camera_matrix: np.ndarray
    dist_coeffs: np.ndarray = field(
        default_factory=lambda: np.array([0.0, 0.0, 0.0, 0.0, 0.0]).reshape((-1, 1)),
    )
    axis_size = 0.02

    def __post_init__(self):
        pass

    def draw_pose_on_img(self, img: np.ndarray, pose_in_cam_frame: np.ndarray):
        img = self.draw_axis(
            img, self.camera_matrix, self.dist_coeffs, pose_in_cam_frame
        )

        return img

    def draw_axis(
        self,
        img: np.ndarray,
        mtx: np.ndarray,
        dist: np.ndarray,
        pose: np.ndarray,
    ):

        s = self.axis_size
        thickness = 2
        R, t = pose[:3, :3], pose[:3, 3]
        K = mtx

        rotV, _ = cv2.Rodrigues(R)
        points = np.float32([[s, 0, 0], [0, s, 0], [0, 0, s], [0, 0, 0]]).reshape(-1, 3)
        axisPoints, _ = cv2.projectPoints(points, rotV, t, K, dist)
        axisPoints = axisPoints.astype(int)

        img = cv2.line(
            img,
            tuple(axisPoints[3].ravel()),
            tuple(axisPoints[0].ravel()),
            (255, 0, 0),
            thickness,
        )
        img = cv2.line(
            img,
            tuple(axisPoints[3].ravel()),
            tuple(axisPoints[1].ravel()),
            (0, 255, 0),
            thickness,
        )

        img = cv2.line(
            img,
            tuple(axisPoints[3].ravel()),
            tuple(axisPoints[2].ravel()),
            (0, 0, 255),
            thickness,
        )
        return img

@dataclass
class OpencvWindow:
    win_name: str

    def __post_init__(self):
        cv2.namedWindow(self.win_name, cv2.WINDOW_NORMAL)

    def show_img(self, img):
        cv2.imshow(self.win_name, img)
        k = cv2.waitKey(1)

        if k == ord("q") or k == 27:
            rospy.signal_shutdown("User exit")


@dataclass
class ImageSubscriber:
    ral: crtk.ral
    camera_image_topic: str
    camera_info_topic: str
    img: np.ndarray = field(default=None, init=False)
    camera_matrix: np.ndarray = field(default=None, init=False)

    def __post_init__(self):
        self.bridge = CvBridge()
        self.image_subscriber = self.ral.subscriber(
            self.camera_image_topic, Image, self._img_callback
        )
        self.info_subscriber = self.ral.subscriber(
            self.camera_info_topic, CameraInfo, self._info_callback
        )

    def _img_callback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            # rospy.logerr(e)
            print(e)

    def _info_callback(self, info_msg):
        projection_matrix = np.array(info_msg.P).reshape((3, 4))
        self.camera_instrinsic = np.array(info_msg.K).reshape((3, 3))
        self.camera_distortion = np.array(info_msg.D).reshape((-1, 1))
        self.camera_matrix = projection_matrix[0:3, 0:3]
        self.camera_frame = info_msg.header.frame_id

    def wait_until_first_frame(self):
        print("Waiting for image topic...")
        timeout = 10
        start = time.time()
        while self.img is None:
            if time.time() - start > timeout:
                raise TimeoutError("Timeout waiting for first frame")

            time.sleep(0.2)


@dataclass
class ArucoMarkerSubscriber:
    marker_arr: List[MarkerPose] = field(init=False, default_factory=list)

    def __post_init__(self):
        self.markers_sub = rospy.Subscriber(
            "/aruco/marker_poses", MarkerPoseArray, self._callback
        )
        self.marker_dict = dict()

    def __len__(self):
        return len(self.marker_dict)

    def _callback(self, msg: MarkerPoseArray):
        self.marker_arr: List[MarkerPose] = msg.markers
        for marker in msg.markers:
            label = marker.id
            pose = convert_msg_to_frame(marker)
            ## additional offset with respect to the Aruco marker in openCV coordinate system
            add_offset = Frame(Rotation.RPY(0, 0, 0), Vector(0, 0, 0))
            pose_new = pose * add_offset
            # change basis from openCV to dVRK
            coord_offset = Frame(Rotation.RPY(0, 0, np.pi), Vector(0, 0, 0))
            pose_in_dvrk = coord_offset * pose_new
            self.marker_dict[label] = pose_in_dvrk

    def create_copy_of_markers_arr(self):
        return self.marker_arr.copy()

    def create_copy_of_markers_dict(self):
        return self.marker_dict.copy()


class arm_custom:
    # simplified jaw class to close gripper
    class __jaw_device:
        def __init__(self, ral, expected_interval, operating_state_instance):
            self.__crtk_utils = crtk.utils(self, ral, expected_interval, operating_state_instance)
            self.__crtk_utils.add_move_jp()
            self.__crtk_utils.add_servo_jp()

    def __init__(self, ral, arm_name, ros_namespace="", expected_interval=0.01):
        # ROS initialization
        if not rospy.get_node_uri():
            rospy.init_node('simplified_arm_class', anonymous=False, log_level=rospy.WARN)
        # populate this class with all the ROS topics we need
        self.__ral = ral.create_child(arm_name)
        self.crtk_utils = crtk.utils(self, self.__ral, expected_interval)
        self.crtk_utils.add_operating_state()
        self.crtk_utils.add_servo_jp()
        self.crtk_utils.add_move_jp()
        self.crtk_utils.add_servo_cp()
        self.crtk_utils.add_move_cp()
        self.crtk_utils.add_measured_js()
        self.crtk_utils.add_measured_cp()
        jaw_ral = self.ral().create_child('/jaw')
        self.jaw = self.__jaw_device(jaw_ral, expected_interval, operating_state_instance=self)
        self.namespace = ros_namespace
        self.name = arm_name

    def ral(self):
        return self.__ral

    def check_connections(self, timeout=5.0):
        self.__ral.check_connections(timeout)


if __name__ == "__main__":
    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-a",
        "--psm-name",
        type=str,
        default="PSM2",
        choices=["PSM1", "PSM2", "PSM3"],
        help="PSM name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace",
    )
    parser.add_argument(
        "-t",
        "--target-tag",
        type=str,
        default=1,
        help="target tag id to reach",
    )
    parser.add_argument(
        "-c",
        "--camera-namespace",
        type=str,
        default='/depstech',
        help="ROS namespace for the camera",
    )
    # extract ros arguments (e.g. __ns:= for namespace)
    argv = crtk.ral.parse_argv(sys.argv[1:])  # skip argv[0], script name
    args = parser.parse_args(argv)
    target_tag = int(args.target_tag)
    camera_image_topic = args.camera_namespace + "/image_rect_color"
    camera_info_topic = args.camera_namespace + "/camera_info"

    ral = crtk.ral("grasp_needle_jaw")
    arm_handle = arm_custom(ral, arm_name=args.psm_name, expected_interval=0.1)
    ral.check_connections()
    cv2.setNumThreads(2)

    img_subscriber = ImageSubscriber(ral, camera_image_topic, camera_info_topic)
    img_subscriber.wait_until_first_frame()
    aruco_marker_sub = ArucoMarkerSubscriber()

    psm_annotator = PoseAnnotatorPSM(img_subscriber.camera_matrix)
    aruco_annotator = PoseAnnotatorAruco(img_subscriber.camera_instrinsic, img_subscriber.camera_distortion)

    # window_name = 'All Annotation dVRK'
    # window = OpencvWindow(window_name)
    # cv2.resizeWindow(window_name, 640, 480)

    # time.sleep(2.0)
    input("Manually move the initial position. Press Enter to Continue.")

    psm_init = copy.deepcopy(arm_handle.measured_cp())

    print('Close the PSM jaw...')
    arm_handle.jaw.move_jp(np.array([0.0])).wait()
    time.sleep(1)
    print('Open the PSM jaw...')
    arm_handle.jaw.move_jp(np.array([0.5])).wait()
    # input('Press Enter to Continue.')
    # arm_handle.jaw.move_jp(np.array([0.0])).wait()

    time.sleep(1.0)

    print('Move to the desired pose for grasping needle ...')

    while not ral.is_shutdown():
        # img = img_subscriber.img
        # print(aruco_marker_sub.create_copy_of_markers_dict())
        if len(aruco_marker_sub) > 0:
            marker_dict = aruco_marker_sub.create_copy_of_markers_dict()
            marker_frame = marker_dict[target_tag]
            offset = Frame(Rotation.RPY(np.pi, 0, 0), Vector(0, 0, 0.02)) ### found the offset!
            tag_in_dvrk_frame = marker_frame * offset
            # tag_in_dvrk_mtx = convert_frame_to_mat(tag_in_dvrk_frame)
            # img = aruco_annotator.draw_pose_on_img(img, tag_in_dvrk_mtx)
        else:
            print(f'Marker {target_tag} is not detected')

        measured_cp = arm_handle.measured_cp()
        T_psm_tip = copy.deepcopy(measured_cp)

        done = False

        while not done:
            T_delta, done = cartesian_interpolate_step(T_psm_tip, tag_in_dvrk_frame, 0.005, 0.005)
            r_delta = T_delta.M.GetRPY()
            T_cmd = Frame()
            T_cmd.p = T_psm_tip.p + T_delta.p
            T_cmd.M = T_psm_tip.M * Rotation.RPY(r_delta[0], r_delta[1], r_delta[2])
            T_psm_tip = T_cmd
            arm_handle.move_cp(T_cmd)
            # time.sleep(0.01)
            # arm_handle.jaw.move_jp(np.array([0.0]))
            # time.sleep(0.01)

        if done:
            break

        # measured_cp_mtx = convert_frame_to_mat(measured_cp)

        # img = psm_annotator.draw_pose_on_img(img, measured_cp_mtx)
        # window.show_img(img)
    time.sleep(2)

    print('Done')

    print('Close the grippers to pick up the needle ....')

    for i in range(3):
        arm_handle.jaw.servo_jp(np.array([-0.6]))
        time.sleep(0.3)

    time.sleep(1)

    # arm_handle.jaw.move_jp(np.array([-0.6])).wait()
    arm_handle.move_cp(psm_init).wait()
    arm_handle.jaw.move_jp(np.array([-0.6])).wait()

    print('Done')

    # cv2.destroyAllWindows()