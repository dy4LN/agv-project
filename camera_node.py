#!/usr/bin/env python3

import cv2
import apriltag
import time
import rospy
import numpy as np
import yaml
import os

from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Int32, Bool

# ============================================================
# ROS NODE INITIALIZATION
# ============================================================

rospy.init_node("agv_vision_node")

# ============================================================
# PARAMETERS (Loaded from ROS param server)
# ============================================================

WIDTH  = rospy.get_param("~width", 640)
HEIGHT = rospy.get_param("~height", 480)
FPS    = rospy.get_param("~fps", 30)

VIDEO_UDP_HOST = rospy.get_param("~video_host", "100.101.250.127")
# VIDEO_UDP_HOST = rospy.get_param("~video_host", "100.90.123.127")
VIDEO_UDP_PORT = rospy.get_param("~video_port", 5000)

TAG_SIZE = rospy.get_param("~tag_size", 0.1651)  # meters
alpha = rospy.get_param("~filter_alpha", 0.3)    # smoothing factor
timeout_seconds = rospy.get_param("~lost_timeout", 0.5)

use_ippe = rospy.get_param("~use_ippe", True)
confidence_threshold = rospy.get_param("~decision_margin_threshold", 30.0)

CAMERA_FRAME = "camera_link"

# ============================================================
# CAMERA CALIBRATION LOADING
# ============================================================

def load_camera_calibration(path):
    """
    Loads camera intrinsic matrix (K) and distortion coefficients (D)
    from a YAML calibration file.
    """
    if not os.path.exists(path):
        rospy.logwarn("Calibration file not found. Using fallback intrinsics.")
        return None, None

    with open(path, 'r') as f:
        calib = yaml.safe_load(f)

    K = np.array(calib["camera_matrix"]["data"]).reshape(3, 3)
    D = np.array(calib["distortion_coefficients"]["data"])

    return K, D

calib_path = rospy.get_param("~calibration_file", "")

if calib_path:
    camera_matrix, dist_coeffs = load_camera_calibration(calib_path)
else:
    camera_matrix, dist_coeffs = None, None

# Fallback intrinsics if calibration file is not provided
if camera_matrix is None:
    fx = 820
    fy = 820
    cx = WIDTH / 2
    cy = HEIGHT / 2

    camera_matrix = np.array([
        [fx,  0, cx],
        [ 0, fy, cy],
        [ 0,  0,  1]
    ], dtype=np.float32)

    dist_coeffs = np.zeros((4, 1), dtype=np.float32)

# ============================================================
# APRILTAG GEOMETRY (Tag corners in local frame)
# ============================================================

half = TAG_SIZE / 2.0

object_points = np.array([
    [-half,  half, 0],
    [ half,  half, 0],
    [ half, -half, 0],
    [-half, -half, 0]
], dtype=np.float32)

# ============================================================
# FILTER STATE (Exponential smoothing)
# ============================================================

filtered_x   = 0.0
filtered_z   = 0.0
filtered_yaw = 0.0

have_filter_state   = False
last_detection_time = 0.0

# ============================================================
# HELPER FUNCTIONS
# ============================================================

def wrap_angle_pi(angle):
    """
    Wrap angle to [-pi, pi] to avoid discontinuities.
    """
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def yaw_to_quaternion(yaw):
    """
    Converts a planar yaw angle into a quaternion.
    Assumes roll = pitch = 0.
    """
    half = 0.5 * yaw
    return 0.0, 0.0, np.sin(half), np.cos(half)

# ============================================================
# ROS PUBLISHERS
# ============================================================

pose_pub    = rospy.Publisher("/pallet_pose_metric", PoseWithCovarianceStamped, queue_size=1)
tag_id_pub  = rospy.Publisher("/detected_tag_id",   Int32,                     queue_size=1)
visible_pub = rospy.Publisher("/pallet_visible",     Bool,                      queue_size=1)

# AprilTag detector instance
detector = apriltag.Detector()

# ============================================================
# CAMERA + VIDEO STREAM SETUP
# ============================================================

# Capture from camera using GStreamer (Jetson optimized pipeline)
cap = cv2.VideoCapture(
    f"v4l2src device=/dev/video0 ! "
    f"video/x-raw, width={WIDTH}, height={HEIGHT}, framerate={FPS}/1 ! "
    f"videoconvert ! appsink",
    cv2.CAP_GSTREAMER
)

# UDP video stream output (compressed H264)
out = cv2.VideoWriter(
    f"appsrc ! videoconvert ! nvvidconv ! "
    f"nvv4l2h264enc insert-sps-pps=true ! "
    f"rtph264pay config-interval=1 pt=96 ! "
    f"udpsink host={VIDEO_UDP_HOST} port={VIDEO_UDP_PORT} sync=false async=false",
    cv2.CAP_GSTREAMER,
    0, FPS, (WIDTH, HEIGHT), True
)

if not cap.isOpened():
    rospy.logerr("Failed to open camera")
    raise SystemExit(1)

rospy.loginfo("AGV Vision Node Started")
rate = rospy.Rate(FPS)

# ============================================================
# MAIN PROCESSING LOOP
# ============================================================

while not rospy.is_shutdown():

    ret, frame = cap.read()
    if not ret:
        rate.sleep()
        continue

    # Convert to grayscale for AprilTag detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = detector.detect(gray)

    # Select closest valid tag (smallest Z distance)
    best  = None
    min_z = float("inf")

    for tag in tags:

        # Reject low-confidence detections
        if tag.decision_margin < confidence_threshold:
            continue

        corners = tag.corners.astype(np.float32)

        # Choose PnP solver
        flags = cv2.SOLVEPNP_IPPE_SQUARE if use_ippe else cv2.SOLVEPNP_ITERATIVE

        success, rvec, tvec = cv2.solvePnP(
            object_points,
            corners,
            camera_matrix,
            dist_coeffs,
            flags=flags
        )

        if not success:
            continue

        z = float(tvec[2][0])

        # Keep closest tag only
        if z < min_z:
            min_z = z
            best  = (tag, rvec, tvec)

    now = time.time()

    # ========================================================
    # VALID DETECTION FOUND
    # ========================================================
    if best is not None:

        tag, rvec, tvec = best
        last_detection_time = now
        visible_pub.publish(True)

        # Extract translation (camera frame)
        x = float(tvec[0][0])  # lateral offset
        z = float(tvec[2][0])  # forward distance

        # ----------------------------------------------------
        # Yaw = bearing angle to tag (NOT tag orientation)
        #
        #   atan2(x, z):
        #     + => tag is left
        #     - => tag is right
        #
        # Used as heading error for controller
        # ----------------------------------------------------
        yaw = float(np.arctan2(x, z))
        yaw = wrap_angle_pi(yaw)

        # Initialize filter or apply exponential smoothing
        if not have_filter_state:
            filtered_x   = x
            filtered_z   = z
            filtered_yaw = yaw
            have_filter_state = True
        else:
            yaw_error    = wrap_angle_pi(yaw - filtered_yaw)
            filtered_yaw = wrap_angle_pi(filtered_yaw + alpha * yaw_error)
            filtered_x   = alpha * x + (1 - alpha) * filtered_x
            filtered_z   = alpha * z + (1 - alpha) * filtered_z

        # Convert yaw → quaternion for ROS message
        qx, qy, qz, qw = yaw_to_quaternion(filtered_yaw)

        # ----------------------------------------------------
        # Build ROS PoseWithCovariance message
        # ----------------------------------------------------
        msg = PoseWithCovarianceStamped()
        msg.header.stamp    = rospy.Time.now()
        msg.header.frame_id = CAMERA_FRAME

        msg.pose.pose.position.x = filtered_x
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = filtered_z

        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # Covariance tuning (distance-dependent uncertainty)
        distance_var = 0.01 + 0.02 * filtered_z
        yaw_var      = 0.02

        msg.pose.covariance = [
            distance_var, 0, 0, 0, 0, 0,
            0, 99999, 0, 0, 0, 0,
            0, 0, distance_var, 0, 0, 0,
            0, 0, 0, 99999, 0, 0,
            0, 0, 0, 0, 99999, 0,
            0, 0, 0, 0, 0, yaw_var
        ]

        pose_pub.publish(msg)
        tag_id_pub.publish(Int32(tag.tag_id))

        # ----------------------------------------------------
        # Visualization overlay (debugging / monitoring)
        # ----------------------------------------------------
        corners_int = tag.corners.astype(int)

        for i in range(4):
            cv2.line(
                frame,
                tuple(corners_int[i]),
                tuple(corners_int[(i + 1) % 4]),
                (0, 255, 0), 2
            )

        cv2.putText(
            frame,
            f"ID:{tag.tag_id}  X:{filtered_x:.3f}m  Z:{filtered_z:.3f}m  Yaw:{np.degrees(filtered_yaw):.1f}deg",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255), 2
        )

    # ========================================================
    # NO DETECTION (timeout handling)
    # ========================================================
    elif (now - last_detection_time) > timeout_seconds:
        have_filter_state = False
        visible_pub.publish(False)

    # Stream frame regardless of detection state
    out.write(frame)
    rate.sleep()

# ============================================================
# CLEANUP
# ============================================================

cap.release()
out.release()
