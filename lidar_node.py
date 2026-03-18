#!/usr/bin/env python3

import rospy
import math
import time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Float32, Bool


class AGVLidar:
    """
    LiDAR processing node for:
    - Collision safety (STOP / SLOW / CLEAR)
    - Front distance estimation
    - Gap detection for navigation
    """

    def __init__(self):

        rospy.init_node("agv_lidar")

        # ============================================================
        # CONSTANTS (all units in meters unless stated)
        # ============================================================

        # Safety thresholds
        self.stop_depth = 0.3429   # immediate stop distance
        self.slow_depth = 0.91     # slow-down threshold

        # Planning limits
        self.plan_distance = 4.0   # max usable LiDAR distance
        self.half_width = 0.457    # half robot width (for collision box)
        self.min_gap_width = 0.914 # minimum width required for valid gap

        # Angular windows (degrees)
        self.front_window_deg = 30  # forward-facing region
        self.plan_window_deg = 90   # planning field of view

        # ============================================================
        # ROS PUBLISHERS
        # ============================================================

        self.status_pub     = rospy.Publisher("/lidar_status", String,  queue_size=10)
        self.front_dist_pub = rospy.Publisher("/lidar_front_distance_m", Float32, queue_size=10)

        self.gap_center_pub = rospy.Publisher("/lidar_gap_center_angle", Float32, queue_size=10)
        self.gap_min_pub    = rospy.Publisher("/lidar_gap_min_angle",    Float32, queue_size=10)
        self.gap_max_pub    = rospy.Publisher("/lidar_gap_max_angle",    Float32, queue_size=10)
        self.has_gap_pub    = rospy.Publisher("/lidar_has_valid_gap",    Bool,    queue_size=10)

        # LiDAR input
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        # State
        self.last_scan = None
        self.last_scan_time = time.time()

        # Main processing loop (10 Hz)
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    # ============================================================
    # SCAN CALLBACK (Pre-processing)
    # ============================================================

    def scan_callback(self, msg):
        """
        Fixes occasional range size mismatch from RPLidar.

        Some drivers produce an extra range entry due to rounding
        of angle_increment. This ensures consistency with expected size.
        """
        expected = int(round((msg.angle_max - msg.angle_min) /
                             msg.angle_increment)) + 1

        if len(msg.ranges) > expected:
            msg.ranges = msg.ranges[:expected]

        self.last_scan = msg
        self.last_scan_time = time.time()

    # ============================================================
    # UTILITY FUNCTIONS
    # ============================================================

    def round_cm(self, value):
        """
        Rounds distance to 2 decimal places (~cm precision).
        Helps reduce noisy output.
        """
        return round(value * 100.0) / 100.0

    def flip_angle_180(self, angle):
        """
        Rotates LiDAR frame by 180 degrees.

        Many LiDARs define 0 rad as "rear".
        This aligns forward direction with robot motion.
        """
        angle += math.pi
        return math.atan2(math.sin(angle), math.cos(angle))

    # ============================================================
    # MAIN PROCESSING LOOP
    # ============================================================

    def timer_callback(self, event):

        # --------------------------------------------------------
        # SAFETY: No scan received yet
        # --------------------------------------------------------
        if self.last_scan is None:
            self.status_pub.publish("UNKNOWN")
            return

        # --------------------------------------------------------
        # SAFETY: Scan timeout (sensor failure or disconnect)
        # --------------------------------------------------------
        if (time.time() - self.last_scan_time) > 0.5:
            self.status_pub.publish("STOP")
            return

        scan = self.last_scan

        # ========================================================
        # RECTANGULAR COLLISION CHECK
        #
        # Projects LiDAR points into robot frame and checks
        # if any fall within a rectangular safety zone.
        # ========================================================

        stop_trigger = False
        slow_trigger = False
        min_front = float("inf")

        angle = scan.angle_min

        for r in scan.ranges:

            if math.isfinite(r) and r > 0.0:

                angle_corrected = self.flip_angle_180(angle)

                # Convert polar → Cartesian (robot frame)
                x = r * math.cos(angle_corrected)  # forward
                y = r * math.sin(angle_corrected)  # lateral

                if x >= 0:  # only consider forward region

                    # Track closest object in forward cone
                    if abs(math.degrees(angle_corrected)) <= self.front_window_deg:
                        min_front = min(min_front, r)

                    # Check within robot width
                    if abs(y) <= self.half_width:

                        if x <= self.stop_depth:
                            stop_trigger = True
                        elif x <= self.slow_depth:
                            slow_trigger = True

            angle += scan.angle_increment

        # Default front distance if nothing detected
        if min_front == float("inf"):
            min_front = self.plan_distance

        min_front = self.round_cm(min_front)
        self.front_dist_pub.publish(min_front)

        # Determine safety state
        if stop_trigger:
            status = "STOP"
        elif slow_trigger:
            status = "SLOW"
        else:
            status = "CLEAR"

        self.status_pub.publish(status)

        # ========================================================
        # GAP DETECTION
        #
        # Finds continuous free-space segments wide enough
        # for the robot to pass through.
        # ========================================================

        free_segments = []
        current_segment = []

        angle = scan.angle_min

        for r in scan.ranges:

            if math.isfinite(r) and r > 0.0:

                angle_corrected = self.flip_angle_180(angle)
                deg = math.degrees(angle_corrected)

                # Only consider forward planning window
                if abs(deg) <= self.plan_window_deg:

                    r_use = min(r, self.plan_distance)

                    # Free space condition
                    if r_use >= self.slow_depth:
                        current_segment.append((angle_corrected, r_use))
                    else:
                        # End of free segment
                        if current_segment:
                            free_segments.append(current_segment)
                            current_segment = []

            angle += scan.angle_increment

        if current_segment:
            free_segments.append(current_segment)

        # --------------------------------------------------------
        # Evaluate best gap
        # --------------------------------------------------------
        best_gap = None
        best_gap_width = 0.0

        for segment in free_segments:

            if len(segment) < 2:
                continue

            angles = [pt[0] for pt in segment]
            dists  = [pt[1] for pt in segment]

            theta_min = angles[0]
            theta_max = angles[-1]
            delta_theta = theta_max - theta_min

            # Conservative width estimate using closest point
            r_min = min(dists)
            width = r_min * delta_theta

            if width >= self.min_gap_width and width > best_gap_width:
                best_gap_width = width
                best_gap = (theta_min, theta_max)

        # --------------------------------------------------------
        # Publish best gap (if found)
        # --------------------------------------------------------
        if best_gap is not None:

            theta_min, theta_max = best_gap
            theta_center = (theta_min + theta_max) / 2.0

            self.gap_center_pub.publish(theta_center)
            self.gap_min_pub.publish(theta_min)
            self.gap_max_pub.publish(theta_max)
            self.has_gap_pub.publish(True)

        else:
            self.has_gap_pub.publish(False)


# ============================================================
# ENTRY POINT
# ============================================================

def main():
    AGVLidar()
    rospy.spin()


if __name__ == "__main__":
    main()
