#!/usr/bin/env python3

from typing import List
import rospy
from dataclasses import dataclass, field
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped
import cv2
from cv_bridge import CvBridge
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
import tf_conversions.posemath as pm


@dataclass
class PoseAnnotator:
    camera_matrix: np.ndarray
    dist_coeffs: np.ndarray = field(
        default_factory=lambda: np.array([0.0, 0.0, 0.0, 0.0, 0.0]).reshape((-1, 1)),
    )
    axis_size = 0.05

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
class RosCamera:
    namespace: str = field(default="/depstech/")
    display_img: bool = False
    image_sub: rospy.Subscriber = None
    img: np.ndarray = field(init=False, default=None)
    camera_instrinsic: np.ndarray = field(init=False, default=None)
    camera_distortion: np.ndarray = field(init=False, default=None)
    camera_frame: str = field(init=False, default=None)

    def __post_init__(self):
        self.image_topic = self.namespace + "image_raw"
        self.camera_info_topic = self.namespace + "camera_info"
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self._image_callback)
        self.cv_bridge = CvBridge()

        self.info_subscriber = rospy.Subscriber(
            self.camera_info_topic, CameraInfo, self._info_callback
        )

    def _image_callback(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.img = cv_image
        except Exception as e:
            pass

    def _info_callback(self, info_msg):
        self.camera_instrinsic = np.array(info_msg.K).reshape((3, 3))
        self.camera_distortion = np.array(info_msg.D).reshape((-1, 1))

        self.camera_frame = info_msg.header.frame_id

    def print_camera_info(self):
        print(f"Streaming image from: {self.image_topic}")
        print(f"Camera instrinsic: {self.camera_instrinsic}")
        print(f"Camera distortion: {self.camera_distortion}")
        print(f"Camera frame: {self.camera_frame}")

    def wait_until_first_img(self, timeout=5.0):
        start_time = rospy.Time.now()
        while self.img is None:
            rospy.sleep(0.1)

            if (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logerr("Timeout waiting for first image")
                exit(1)

        return self.img


@dataclass
class AlvarMarkerSubscriber:
    marker_arr: List[AlvarMarker] = field(init=False, default_factory=list)

    def __post_init__(self):
        self.markers_sub = rospy.Subscriber(
            "/ar_pose_marker", AlvarMarkers, self._callback
        )

    def __len__(self):
        return len(self.marker_arr)

    def _callback(self, msg: AlvarMarkers):
        self.marker_arr: List[AlvarMarker] = msg.markers

    def create_copy_of_markers_arr(self):
        return self.marker_arr.copy()


def convert_pose_msg_to_matrix(pose_msg: PoseStamped):
    return pm.toMatrix(pm.fromMsg(pose_msg))


def main():
    rospy.init_node("AlvarMarkerPoseAnnotator")
    rospy.loginfo(
        "Starting AlvarMarkerPoseAnnotator. Coordinates frames will be overlaid on the camera image."
    )
    rospy.loginfo("Press 'q' or 'esc' to exit the program.")

    camera_handle = RosCamera(display_img=True)
    window = OpencvWindow(f"Annotating {camera_handle.image_topic}")
    camera_handle.wait_until_first_img()
    camera_handle.print_camera_info()
    pose_annotator = PoseAnnotator(
        camera_handle.camera_instrinsic, camera_handle.camera_distortion
    )

    alvar_marker_sub = AlvarMarkerSubscriber()

    while not rospy.is_shutdown():
        img = camera_handle.img

        if len(alvar_marker_sub) > 0:
            marker_arr = alvar_marker_sub.create_copy_of_markers_arr()

            for marker in marker_arr:
                pose_in_cam_frame = convert_pose_msg_to_matrix(marker.pose.pose)
                img = pose_annotator.draw_pose_on_img(img, pose_in_cam_frame)

        window.show_img(img)

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
