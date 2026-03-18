#!/usr/bin/env python3

import rospy
import serial
import math
import threading
import time
import json
import socket

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry


# ============================================================
# CONFIGURATION
# ============================================================

SERIAL_PORT = "/dev/teensy"
BAUD_RATE = 460800

WHEEL_RADIUS_M = 0.0333375
WHEEL_BASE_M = 0.1190625

MAX_RPM = 160.0
MIN_RPM = 30.0

CMD_TIMEOUT_SEC = 0.5

# Telemetry networking
LAPTOP_IP = "100.101.250.127"
# LAPTOP_IP = "100.90.123.127"
LAPTOP_PORT = 9000
HZ = 15
SER_TIMEOUT = 0.05

DEBUG_PRINT = False

# Velocity sanity limits (protect against bad telemetry spikes)
MAX_LINEAR_MPS = 0.6
MAX_ANGULAR_RPS = 8.1

MAX_LINEAR_JUMP = 0.6
MAX_ANGULAR_JUMP = 8.1


# ============================================================
# MOTOR INTERFACE NODE
#
# Responsibilities:
# - Send velocity commands to Teensy (RPM control)
# - Read telemetry from Teensy (serial)
# - Publish wheel odometry
# - Stream telemetry to laptop (UDP JSON)
# ============================================================

class MotorInterfaceNode:

    def __init__(self):

        rospy.init_node("agv_motor_interface_node")

        # --------------------------------------------------------
        # Hardware Interfaces
        # --------------------------------------------------------
        self.ser = self.open_serial()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # --------------------------------------------------------
        # Command State
        # --------------------------------------------------------
        self.linear = 0.0
        self.angular = 0.0
        self.lift = 0
        self.last_cmd_time = time.time()

        # --------------------------------------------------------
        # Telemetry State (latest values from Teensy)
        # --------------------------------------------------------
        self.latest_tel = {
            "linearVelocity": 0.0,
            "angularVelocity": 0.0,
            "rpmL": 0.0,
            "rpmR": 0.0,
            "encHzL": 0.0,
            "encHzR": 0.0,
            "imuHz": 0.0,
            "pwmL": 0,
            "pwmR": 0,
            "targetRPM_L": 0.0,
            "targetRPM_R": 0.0,
            "actState": 0,
            "ts_us": 0,
        }

        # Last valid velocities (used for sanity filtering)
        self.last_valid_linear = 0.0
        self.last_valid_angular = 0.0

        # Thread safety
        self.lock = threading.Lock()
        self.reconnect_lock = threading.Lock()

        # Telemetry packet sequence
        self.seq = 0

        # --------------------------------------------------------
        # ROS INTERFACES
        # --------------------------------------------------------
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        rospy.Subscriber("/cmd_actuator", Int32, self.actuator_callback)

        self.odom_pub = rospy.Publisher("/wheel/odometry", Odometry, queue_size=10)

        # Timers
        rospy.Timer(rospy.Duration(0.02), self.update)                    # 50 Hz control loop
        rospy.Timer(rospy.Duration(1.0 / float(HZ)), self.telemetry_loop) # telemetry rate

        # Background serial reader
        threading.Thread(target=self.serial_reader, daemon=True).start()

        rospy.loginfo("AGV Motor Interface + Telemetry Node started.")

    # ============================================================
    # SERIAL SETUP
    # ============================================================

    def open_serial(self):
        """
        Attempts to open serial connection to Teensy.
        Retries until successful.
        """
        while not rospy.is_shutdown():
            try:
                ser = serial.Serial(
                    SERIAL_PORT,
                    BAUD_RATE,
                    timeout=SER_TIMEOUT,
                    dsrdtr=False,
                    rtscts=False
                )
                time.sleep(0.1)
                ser.reset_input_buffer()
                ser.reset_output_buffer()
                rospy.loginfo("Connected to Teensy.")
                return ser

            except serial.SerialException:
                rospy.logwarn("Waiting for Teensy...")
                time.sleep(1)

    def reconnect_serial(self):
        """
        Handles serial reconnection safely (thread-protected).
        """
        with self.reconnect_lock:
            rospy.logwarn("Serial connection lost. Attempting to reconnect...")
            try:
                self.ser.close()
            except Exception:
                pass

            self.ser = self.open_serial()
            rospy.loginfo("Reconnected to Teensy.")

    # ============================================================
    # ROS CALLBACKS
    # ============================================================

    def cmd_vel_callback(self, msg):
        """
        Receives velocity command from navigation/manual control.
        """
        self.linear = msg.linear.x
        self.angular = msg.angular.z
        self.last_cmd_time = time.time()

    def actuator_callback(self, msg):
        """
        Receives actuator command (e.g., lift control).
        """
        self.lift = int(msg.data)

    # ============================================================
    # RPM CONVERSION
    # ============================================================

    def clamp_rpm(self, rpm):
        """Limits RPM to motor constraints."""
        if rpm > MAX_RPM:
            return MAX_RPM
        if rpm < -MAX_RPM:
            return -MAX_RPM
        return rpm

    def apply_min_deadband(self, rpm):
        """
        Enforces minimum RPM to overcome motor deadband.
        """
        if rpm == 0:
            return 0.0
        if 0 < abs(rpm) < MIN_RPM:
            return MIN_RPM if rpm > 0 else -MIN_RPM
        return rpm

    def convert_to_rpm(self, linear, angular):
        """
        Converts linear/angular velocity to differential wheel RPM.
        """
        vL = linear - (angular * WHEEL_BASE_M / 2.0)
        vR = linear + (angular * WHEEL_BASE_M / 2.0)

        rpmL = (vL / (2.0 * math.pi * WHEEL_RADIUS_M)) * 60.0
        rpmR = (vR / (2.0 * math.pi * WHEEL_RADIUS_M)) * 60.0

        rpmL = self.apply_min_deadband(self.clamp_rpm(rpmL))
        rpmR = self.apply_min_deadband(self.clamp_rpm(rpmR))

        return rpmL, rpmR

    # ============================================================
    # VELOCITY SANITY CHECK
    # ============================================================

    def velocity_is_valid(self, linear, angular):
        """
        Rejects invalid or unrealistic telemetry values.
        """
        if not (math.isfinite(linear) and math.isfinite(angular)):
            return False

        if abs(linear) > MAX_LINEAR_MPS:
            return False

        if abs(angular) > MAX_ANGULAR_RPS:
            return False

        if abs(linear - self.last_valid_linear) > MAX_LINEAR_JUMP:
            return False

        if abs(angular - self.last_valid_angular) > MAX_ANGULAR_JUMP:
            return False

        return True

    # ============================================================
    # ODOMETRY PUBLISHER
    # ============================================================

    def publish_wheel_odometry(self):
        """
        Publishes velocity-only odometry from Teensy telemetry.
        """
        with self.lock:
            linear = self.latest_tel["linearVelocity"]
            angular = self.latest_tel["angularVelocity"]

        # Apply sanity filtering
        if not self.velocity_is_valid(linear, angular):
            rospy.logwarn_throttle(1.0, "Rejected insane velocity from Teensy")
            linear = self.last_valid_linear
            angular = self.last_valid_angular
        else:
            self.last_valid_linear = linear
            self.last_valid_angular = angular

        odom = Odometry()

        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # No pose estimation here (only velocity)
        odom.pose.pose.orientation.w = 1.0

        odom.twist.twist.linear.x = linear
        odom.twist.twist.angular.z = angular

        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[35] = 0.02

        self.odom_pub.publish(odom)

    # ============================================================
    # MOTOR COMMAND LOOP
    # ============================================================

    def update(self, event):

        # Timeout safety (stop robot if commands stop)
        if (time.time() - self.last_cmd_time) > CMD_TIMEOUT_SEC:
            linear = 0.0
            angular = 0.0
        else:
            linear = self.linear
            angular = self.angular

        rpmL, rpmR = self.convert_to_rpm(linear, angular)

        packet = f"RPM {rpmL:.2f} {rpmR:.2f} {self.lift}\n"

        try:
            self.ser.write(packet.encode())
        except serial.SerialException:
            rospy.logerr("Serial write failed. Reconnecting...")
            self.reconnect_serial()

        self.publish_wheel_odometry()

    # ============================================================
    # SERIAL TELEMETRY READER
    # ============================================================

    def serial_reader(self):
        """
        Continuously reads telemetry packets from Teensy.
        Expected format:
        TEL,... (comma-separated values)
        """
        while not rospy.is_shutdown():
            try:
                line = self.ser.readline().decode(errors="ignore").strip()

                if not line:
                    continue
                if not line.startswith("TEL,"):
                    continue

                parts = line.split(",")
                if len(parts) != 14:
                    continue

                (_, rpmL, rpmR,
                 encHzL, encHzR,
                 targetL, targetR,
                 pwmL, pwmR,
                 linearVel, angularVel,
                 imuHz,
                 actState,
                 ts_us) = parts

                with self.lock:
                    self.latest_tel["rpmL"] = float(rpmL)
                    self.latest_tel["rpmR"] = float(rpmR)
                    self.latest_tel["encHzL"] = float(encHzL)
                    self.latest_tel["encHzR"] = float(encHzR)
                    self.latest_tel["targetRPM_L"] = float(targetL)
                    self.latest_tel["targetRPM_R"] = float(targetR)
                    self.latest_tel["pwmL"] = int(pwmL)
                    self.latest_tel["pwmR"] = int(pwmR)
                    self.latest_tel["linearVelocity"] = float(linearVel)
                    self.latest_tel["angularVelocity"] = float(angularVel)
                    self.latest_tel["imuHz"] = float(imuHz)
                    self.latest_tel["actState"] = int(actState)
                    self.latest_tel["ts_us"] = int(ts_us)

            except serial.SerialException:
                rospy.logwarn("Serial read failed. Waiting for reconnect...")
                time.sleep(0.5)
            except Exception:
                continue

    # ============================================================
    # TELEMETRY PACKET BUILDER
    # ============================================================

    def build_packet(self):
        """
        Builds structured JSON telemetry packet for laptop.
        """
        now = time.time()

        with self.lock:
            tel = self.latest_tel.copy()

        pkt = {
            "schema": 1,
            "seq": self.seq,
            "ts": now,
            "state": "MANUAL",
            "fault": "NONE",

            "motion": {
                "cmd": {
                    "linear_mps": round(self.linear, 3),
                    "angular_rps": round(self.angular, 3),
                },
                "actual": {
                    "linear_mps": round(tel["linearVelocity"], 3),
                    "angular_rps": round(tel["angularVelocity"], 3),
                }
            },

            "speed": {
                "linear_mps": round(tel["linearVelocity"], 3),
                "angular_rps": round(tel["angularVelocity"], 3),
            },

            "motors": {
                "left": {
                    "rpm": tel["rpmL"],
                    "target_rpm": tel["targetRPM_L"],
                    "pwm": tel["pwmL"],
                    "enc_hz": tel["encHzL"],
                },
                "right": {
                    "rpm": tel["rpmR"],
                    "target_rpm": tel["targetRPM_R"],
                    "pwm": tel["pwmR"],
                    "enc_hz": tel["encHzR"],
                }
            },

            "peripherals": {
                "imu_hz": tel["imuHz"]
            },

            "actuator": {
                "state": tel["actState"]
            },

            "debug": {
                "teensy_timestamp_us": tel["ts_us"]
            }
        }

        return pkt

    # ============================================================
    # TELEMETRY TRANSMISSION LOOP
    # ============================================================

    def telemetry_loop(self, event):

        pkt = self.build_packet()
        data = json.dumps(pkt, separators=(",", ":")).encode("utf-8")

        try:
            self.sock.sendto(data, (LAPTOP_IP, LAPTOP_PORT))
        except Exception:
            pass

        self.seq += 1


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":
    try:
        MotorInterfaceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
