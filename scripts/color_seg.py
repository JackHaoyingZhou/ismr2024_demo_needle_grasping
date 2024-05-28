import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import matplotlib.pyplot as plt


class ColorSeg:
    def __init__(self, camera_ns='/depstech',show_image=True):
        self.bridge = CvBridge()
        # self.image_sub = rospy.Subscriber('/depstech/image_raw', Image, self.image_callback)
        self.image_sub = rospy.Subscriber('/realsense/image_raw', Image, self.image_callback)
        # self.cam_info_sub = rospy.Subscriber('/depstech/camera_info', CameraInfo, self.cam_info_callback)
        self.cam_info_sub = rospy.Subscriber('/realsense/camera_info', CameraInfo, self.cam_info_callback)
        self.show_img = show_image
        self.camera_matrix = None
        self.dist_coeffs = None
        self.cv_image = None

    def cam_info_callback(self, msg):
        intrinsic_mtx = np.array(msg.K).reshape((3, 3))
        distortion_vec = np.array(msg.D).reshape((-1, 1))
        self.camera_matrix = intrinsic_mtx
        self.dist_coeffs = distortion_vec

    def image_callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            lower_bound = np.array([50, 70, 85])
            upper_bound = np.array([115, 125, 130])
            mask = cv2.inRange(self.cv_image, lower_bound, upper_bound)
            img_seg = cv2.bitwise_and(self.cv_image, self.cv_image, mask=mask)

            if self.show_img:
                cv2.imshow('Original Image', self.cv_image)
                cv2.imshow('Segmented Image', img_seg)
                cv2.imshow('Mask', mask)

                k = cv2.waitKey(1)
                if k == 27 or k == ord('q'):
                    cv2.destroyAllWindows()
                    rospy.signal_shutdown("Image Window Closed")


        except CvBridgeError as e:
            rospy.logerr(e)

    def img_color_seg(self):
        cv_img = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB)
        lower_bound = np.array([0, 0, 120])
        upper_bound = np.array([0, 255, 255])
        mask = cv2.inRange(cv_img, lower_bound, upper_bound)
        img_seg = cv2.bitwise_and(cv_img, cv_img, mask=mask)

        if self.show_img:
            cv2.imshow('Original Image', cv_img)
            cv2.imshow('Segmented Image', img_seg)
            cv2.imshow('Mask', mask)
            cv2.waitKey(1)


if __name__=='__main__':
    rospy.init_node('color_segmentation', anonymous=True)
    Img_Seg = ColorSeg()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

    cv2.destroyAllWindows()

