#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from std_msgs.msg import String, Bool, Float32, Int32
from tf2_msgs.msg import TFMessage


# =====================================================
# PID CONTROLLER
# =====================================================

class PIDController:
    """
    Standard PID controller with anti-windup and output clamping.
    """

    def __init__(self, kp, ki, kd, output_min, output_max, integral_max=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_max = integral_max

        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None

    def compute(self, error, now=None):

        if now is None:
            now = rospy.Time.now().to_sec()

        if self._prev_time is None:
            dt = 0.05
        else:
            dt = now - self._prev_time
            if dt <= 0.0:
                dt = 1e-6

        p = self.kp * error

        self._integral += error * dt
        self._integral = max(-self.integral_max,
                             min(self.integral_max, self._integral))
        i = self.ki * self._integral

        if self._prev_time is None:
            d = 0.0
        else:
            d = self.kd * (error - self._prev_error) / dt

        output = p + i + d
        output = max(self.output_min, min(self.output_max, output))

        self._prev_error = error
        self._prev_time = now

        return output


# =====================================================
# LINEAR SPEED CONTROLLER
# =====================================================

class LinearSpeedController:
    """
    Computes forward speed based on distance and heading alignment.
    """

    def __init__(self, lin_min, lin_max, dist_gain=0.15, heading_scale_k=2.5):
        self.lin_min = lin_min
        self.lin_max = lin_max
        self.dist_gain = dist_gain
        self.heading_scale_k = heading_scale_k

    def compute(self, distance, heading_error):

        base = self.lin_min + self.dist_gain * distance
        base = min(base, self.lin_max)

        # Reduce speed when heading error is large
        scale = math.exp(-self.heading_scale_k * abs(heading_error))
        speed = base * scale

        return max(self.lin_min, min(self.lin_max, speed))


# =====================================================
# AGV NAVIGATION NODE
# =====================================================

class AGVNavigation:

    def __init__(self):

        rospy.init_node("agv_navigation")

        # =============================
        # SPEED CONSTRAINTS
        # =============================
        self.lin_min = 0.05
        self.lin_max = 0.10
        self.lin_slow = 0.06
        self.ang_min = 0.3
        self.ang_max = 2.0

        # =============================
        # PARAMETERS
        # =============================
        self.goal_tolerance = 0.2
        self.final_approach_dist = 2.0

        # =============================
        # CONTROLLERS
        # =============================
        self.heading_pid = PIDController(
            kp=0.6,
            ki=0.02,
            kd=0.08,
            output_min=-self.ang_max,
            output_max=self.ang_max,
            integral_max=0.5,
        )

        self.linear_ctrl = LinearSpeedController(
            lin_min=self.lin_min,
            lin_max=self.lin_max,
            dist_gain=0.02,
            heading_scale_k=1.5,
        )

        # =============================
        # STATE
        # =============================
        self.mission_state = "MANUAL"

        self.pose_x = 0.0
        self.pose_y = 0.0
        self.yaw = 0.0
        self._pose_received = False

        self._map_odom = None
        self._odom_base = None

        self.waypoints = []
        self.current_index = 0
        self.path_received = False

        self.desired_tag_id = None

        self.nav_state = "IDLE"

        self.dropoff_sent = False
        self.dropoff_start_time = None
        self.dropoff_retract_duration = 3.0

        # =============================
        # LIDAR STATE
        # =============================
        self.lidar_status = "UNKNOWN"
        self.lidar_front_dist = float("inf")
        self.lidar_has_gap = False
        self.lidar_gap_center = 0.0

        # =============================
        # PUBLISHERS
        # =============================
        self.cmd_pub = rospy.Publisher("/nav_cmd_vel", Twist, queue_size=10)
        self.actuator_pub = rospy.Publisher("/nav_cmd_actuator", Int32, queue_size=10)
        self.delivery_done_pub = rospy.Publisher("/delivery_done", Bool, queue_size=10)
        self.delivery_location_received_pub = rospy.Publisher(
            "/delivery_location_received", Bool, queue_size=10
        )

        # =============================
        # SUBSCRIBERS
        # =============================
        rospy.Subscriber("/tf", TFMessage, self.tf_callback)

        rospy.Subscriber("/mission_state", String, self.mission_callback)
        rospy.Subscriber("/waypoints", Path, self.path_callback)
        rospy.Subscriber("/desired_tag_id", Int32, self.desired_tag_callback)

        rospy.Subscriber("/lidar_status", String, self.lidar_status_callback)
        rospy.Subscriber("/lidar_front_distance_m", Float32, self.lidar_front_dist_callback)
        rospy.Subscriber("/lidar_has_valid_gap", Bool, self.lidar_has_gap_callback)
        rospy.Subscriber("/lidar_gap_center_angle", Float32, self.lidar_gap_center_callback)

        rospy.Timer(rospy.Duration(0.05), self.update)
        rospy.Timer(rospy.Duration(1.0), self.debug_output)

        rospy.loginfo("AGV Navigation node started (map->odom->base_link pose).")

    # =====================================================
    # GAP-ASSIST (ADDED)
    # =====================================================

    def blend_heading_with_gap(self, heading_error):
        """
        Blends global path heading with LiDAR gap direction.

        This creates smooth obstacle avoidance without
        overriding path tracking.
        """

        if not self.lidar_has_gap:
            return heading_error

        gap_heading = self.wrap_angle(self.lidar_gap_center)

        w_path = 0.7
        w_gap = 0.3

        blended = self.wrap_angle(
            w_path * heading_error + w_gap * gap_heading
        )

        return blended

    # =====================================================
    # TF CALLBACK (POSE ESTIMATION)
    # =====================================================

    def tf_callback(self, msg):

        for t in msg.transforms:
            fid = t.header.frame_id
            cid = t.child_frame_id
            q = t.transform.rotation
            tx = t.transform.translation.x
            ty = t.transform.translation.y

            siny = 2 * (q.w * q.z + q.x * q.y)
            cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny, cosy)

            if fid == "map" and cid == "odom":
                self._map_odom = (tx, ty, yaw)
            elif fid == "odom" and cid == "base_link":
                self._odom_base = (tx, ty, yaw)

        if self._map_odom and self._odom_base:
            mx, my, myaw = self._map_odom
            ox, oy, oyaw = self._odom_base

            self.pose_x = mx + ox * math.cos(myaw) - oy * math.sin(myaw)
            self.pose_y = my + ox * math.sin(myaw) + oy * math.cos(myaw)
            self.yaw = self.wrap_angle(myaw + oyaw)

            self._pose_received = True

    # =====================================================
    # DEBUG
    # =====================================================

    def debug_output(self, event):

        rospy.loginfo(
            f"[NAV DEBUG] state={self.nav_state} | "
            f"mission={self.mission_state} | "
            f"pose=({self.pose_x:.2f},{self.pose_y:.2f},{math.degrees(self.yaw):.1f}) | "
            f"lidar={self.lidar_status}"
        )

    # =====================================================
    # CALLBACKS
    # =====================================================

    def mission_callback(self, msg):
        self.mission_state = msg.data

    def desired_tag_callback(self, msg):
        self.desired_tag_id = msg.data if msg.data >= 0 else None

    def path_callback(self, msg):

        self.waypoints = [
            (p.pose.position.x, p.pose.position.y)
            for p in msg.poses
        ]

        self.current_index = 0
        self.path_received = len(self.waypoints) > 0

        if self.path_received:
            self.delivery_location_received_pub.publish(True)

        self.heading_pid.reset()
        self.dropoff_sent = False
        self.dropoff_start_time = None

    def lidar_status_callback(self, msg):
        self.lidar_status = msg.data

    def lidar_front_dist_callback(self, msg):
        self.lidar_front_dist = msg.data

    def lidar_has_gap_callback(self, msg):
        self.lidar_has_gap = msg.data

    def lidar_gap_center_callback(self, msg):
        self.lidar_gap_center = msg.data

    # =====================================================
    # MAIN LOOP
    # =====================================================

    def update(self, event):

        cmd = Twist()

        if not self._pose_received:
            self.cmd_pub.publish(cmd)
            return

        if self.mission_state != "NAVIGATION":
            self.heading_pid.reset()
            self.cmd_pub.publish(cmd)
            return

        if not self.path_received:
            self.cmd_pub.publish(cmd)
            return

        if self.current_index >= len(self.waypoints):
            self.cmd_pub.publish(cmd)
            return

        if self.lidar_status in ["STOP", "UNKNOWN"]:
            self.cmd_pub.publish(cmd)
            return

        goal_x, goal_y = self.waypoints[self.current_index]

        dx = goal_x - self.pose_x
        dy = goal_y - self.pose_y
        distance = math.hypot(dx, dy)

        goal_heading = math.atan2(dy, dx)
        heading_error = self.wrap_angle(goal_heading - self.yaw)

        # =============================
        # GAP ASSIST (ADDED)
        # =============================
        if self.lidar_status == "SLOW" or self.lidar_has_gap:
            heading_error = self.blend_heading_with_gap(heading_error)

        last_index = (
            min(self.desired_tag_id, len(self.waypoints) - 1)
            if self.desired_tag_id is not None
            else len(self.waypoints) - 1
        )

        if distance < self.goal_tolerance:

            if self.current_index < last_index:
                self.current_index += 1
                self.heading_pid.reset()
                return
            else:
                self.nav_state = "DROP_OFF"
        else:
            self.nav_state = "PATH_FOLLOW"

        if self.nav_state == "PATH_FOLLOW":

            cmd.angular.z = self.compute_angular(heading_error)

            if abs(heading_error) < math.radians(45):

                linear_speed = self.linear_ctrl.compute(distance, heading_error)

                if self.lidar_status == "SLOW":
                    linear_speed = min(linear_speed, self.lin_slow)

                cmd.linear.x = linear_speed

            else:
                cmd.linear.x = 0.0

        elif self.nav_state == "DROP_OFF":

            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

            now = rospy.Time.now().to_sec()

            if self.dropoff_start_time is None:
                self.dropoff_start_time = now

            if now - self.dropoff_start_time < self.dropoff_retract_duration:
                self.actuator_pub.publish(Int32(-1))
            else:
                self.actuator_pub.publish(Int32(0))
                if not self.dropoff_sent:
                    self.delivery_done_pub.publish(True)
                    self.dropoff_sent = True

        self.cmd_pub.publish(cmd)

    # =====================================================
    # CONTROLLER
    # =====================================================

    def compute_angular(self, error):

        now = rospy.Time.now().to_sec()
        speed = self.heading_pid.compute(error, now)

        if abs(speed) < self.ang_min and abs(error) > 0.05:
            speed = self.ang_min * math.copysign(1, speed)

        return speed

    # =====================================================
    # UTIL
    # =====================================================

    def wrap_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))


# =====================================================
# ENTRY POINT
# =====================================================

if __name__ == "__main__":
    try:
        AGVNavigation()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
