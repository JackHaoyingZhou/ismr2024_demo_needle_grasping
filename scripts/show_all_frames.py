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
dynamic_path = os.path.abspath(__file__ + "/../../")
# print(dynamic_path)
sys.path.append(dynamic_path)


@dataclass
class PoseAnnotatorPSM:
    camera_matrix: np.ndarray
    cam_T_base: np.ndarray
    dist_coeffs: np.ndarray = field(
        default_factory=lambda: np.array([0.0, 0.0, 0.0, 0.0, 0.0]).reshape((-1, 1)),
        init=False,
    )

    def __post_init__(self):
        print(self.dist_coeffs.shape)

    def draw_pose_on_img(self, img: np.ndarray, local_measured_cp: np.ndarray):
        offset = np.eye(4)
        # offset[0, 3] = -0.008 # z dir
        # offset[1, 3] =  0.005 # y dir
        # offset[2, 3] = 0.008 # x dir
        pose = self.cam_T_base @ local_measured_cp @ offset

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

    def __len__(self):
        return len(self.marker_arr)

    def _callback(self, msg: MarkerPoseArray):
        self.marker_arr: List[MarkerPose] = msg.markers

    def create_copy_of_markers_arr(self):
        return self.marker_arr.copy()


def load_hand_eye_calibration(json_file: str) -> np.ndarray:
    '''
    Load hand eye calibration file for open CV
    :param json_file: the file path
    :return: 4x4 transformation matrix
    '''
    with open(json_file, "r") as f:
        data = json.load(f)

    cam_T_robot_base = np.array(data['base-frame']['transform']).reshape(4, 4)
    return cam_T_robot_base


class arm_custom:
    # simplified jaw class to close gripper
    class __jaw_device:
        def __init__(self, ral, expected_interval, operating_state_instance):
            self.__crtk_utils = crtk.utils(self, ral, expected_interval, operating_state_instance)
            self.__crtk_utils.add_move_jp()
            self.__crtk_utils.add_servo_jp()

    class Local:
        def __init__(self, ral, expected_interval, operating_state_instance):
            self.crtk_utils = crtk.utils(self, ral, expected_interval, operating_state_instance)
            self.crtk_utils.add_measured_cp()
            self.crtk_utils.add_forward_kinematics()

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
        psm_local = self.ral().create_child("local")
        self.local = self.Local(psm_local, expected_interval, operating_state_instance=self)
        self.name = arm_name
        base_frame_topic = "/{}/set_base_frame".format(self.namespace)
        self._set_base_frame_pub = self.ral().publisher(base_frame_topic, PoseStamped, queue_size=1, latch=True)

    def ral(self):
        return self.__ral

    def check_connections(self, timeout=5.0):
        self.__ral.check_connections(timeout)


if __name__ == "__main__":
    # extract ros arguments (e.g. __ns:= for namespace)
    argv = crtk.ral.parse_argv(sys.argv[1:])  # skip argv[0], script name

    hand_eye_json_path = os.path.join(dynamic_path, 'test_data', 'PSM2-registration-open-cv.json')

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
        "-f",
        "--hand-eye-json",
        type=str,
        default=hand_eye_json_path,
        help="hand-eye calibration matrix in JSON format using OpenCV coordinate system",
    )
    parser.add_argument(
        "-c",
        "--camera-namespace",
        type=str,
        default='/depstech',
        help="ROS namespace for the camera",
    )
    args = parser.parse_args(argv)

    camera_image_topic = args.camera_namespace + "/image_rect_color"
    camera_info_topic = args.camera_namespace + "/camera_info"

    ral = crtk.ral("visualize_all_frames")
    arm_handle = arm_custom(ral, arm_name=args.psm_name, expected_interval=0.1)
    ral.check_connections()
    cv2.setNumThreads(2)
    cam_T_robot_base = load_hand_eye_calibration(args.hand_eye_json)

    img_subscriber = ImageSubscriber(ral, camera_image_topic, camera_info_topic)
    img_subscriber.wait_until_first_frame()
    aruco_marker_sub = ArucoMarkerSubscriber()

    psm_annotator = PoseAnnotatorPSM(img_subscriber.camera_matrix, cam_T_robot_base)
    aruco_annotator = PoseAnnotatorAruco(img_subscriber.camera_instrinsic, img_subscriber.camera_distortion)

    window_name = 'All Annotation'
    window = OpencvWindow(window_name)
    # cv2.resizeWindow(window_name, 640, 480)

    while not ral.is_shutdown():

        img = img_subscriber.img

        if len(aruco_marker_sub) > 0:
            marker_arr = aruco_marker_sub.create_copy_of_markers_arr()

            for marker in marker_arr:
                pose_in_cam_frame = pm.toMatrix(pm.fromMsg(marker.pose))
                img = aruco_annotator.draw_pose_on_img(img, pose_in_cam_frame)

        local_measured_cp = pm.toMatrix(arm_handle.local.measured_cp())

        img = psm_annotator.draw_pose_on_img(img, local_measured_cp)
        window.show_img(img)

    cv2.destroyAllWindows()