import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
import cv2
import numpy as np
import tf

# Load the predefined dictionary
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

# Initialize the detector parameters using default values
parameters = cv2.aruco.DetectorParameters_create()

# Load the camera calibration parameters (camera matrix and distortion coefficients)
camera_matrix = np.array([[473.3367332805383, 0, 322.566293310943], 
                          [0, 474.3457171142228, 237.8821653705013], 
                          [0, 0, 1]])
dist_coeffs = np.array([0.04628682088784457, -0.0804026062935555, 0.003725307136288697, 0.002703884650555371, 0])

# Marker length (in meters)
markerLength = 0.03

# Initialize ROS node
rospy.init_node('aruco_detector', anonymous=True)

# Publisher for pose
pose_pub = rospy.Publisher('/aruco_pose', PoseStamped, queue_size=10)

# Capture video from the default camera
cap = cv2.VideoCapture(0,cv2.CAP_V4L2)

width = 1280
height = 720
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

def average_poses(rvecs, tvecs):
    rvec_avg = np.mean(rvecs, axis=0)
    tvec_avg = np.mean(tvecs, axis=0)
    return rvec_avg, tvec_avg

def publish_pose(rvec, tvec):
    pose = PoseStamped()
    pose.header = Header()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "camera"

    pose.pose.position.x = tvec[0][0]
    pose.pose.position.y = tvec[0][1]
    pose.pose.position.z = tvec[0][2]

    # Convert rotation vector to quaternion
    rotation_matrix, _ = cv2.Rodrigues(rvec)
    quaternion = tf.transformations.quaternion_from_matrix(
        np.vstack((np.hstack((rotation_matrix, [[0], [0], [0]])), [0, 0, 0, 1]))
    )

    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]

    pose_pub.publish(pose)

while not rospy.is_shutdown() and cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Convert frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect markers
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # If markers are detected
    if ids is not None:
        # Draw detected markers
        frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # Estimate pose of each marker
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeffs)

        # Average poses for improved accuracy
        rvec_avg, tvec_avg = average_poses(rvecs, tvecs)

        # Draw axis for the averaged pose
        frame = cv2.aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvec_avg, tvec_avg, 0.1)

        # Publish the averaged pose
        publish_pose(rvec_avg, tvec_avg)

    # Display the resulting frame
    cv2.imshow('frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
