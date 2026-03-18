#!/usr/bin/env python3

import rospy
import socket

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, String, Bool

# ============================================================
# CONFIGURATION
# ============================================================

TCP_PORT = 8765
BUFFER_SIZE = 1024


# ============================================================
# MANUAL CONTROL BRIDGE
#
# Receives TCP commands (from laptop/app) and translates them
# into ROS topics for robot control.
# ============================================================

class ManualControlBridge:

    def __init__(self):

        rospy.init_node("agv_manual_control_node")

        # ----------------------------------------------------
        # ROS PUBLISHERS
        # ----------------------------------------------------
        self.vel_pub = rospy.Publisher(
            "/manual_cmd_vel", Twist, queue_size=10
        )

        self.act_pub = rospy.Publisher(
            "/manual_cmd_actuator", Int32, queue_size=10
        )

        self.tag_pub = rospy.Publisher(
            "/input_tag_id", Int32, queue_size=10
        )

        self.auto_pub = rospy.Publisher(
            "/auto_state", Bool, queue_size=10
        )

        self.state_pub = rospy.Publisher(
            "/requested_state", String, queue_size=10
        )

        # ----------------------------------------------------
        # TCP SERVER SETUP
        #
        # Listens for a single client (laptop/app) and streams
        # newline-delimited commands.
        # ----------------------------------------------------
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Allow immediate rebinding after restart
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        self.sock.bind(("0.0.0.0", TCP_PORT))
        self.sock.listen(1)

        rospy.loginfo(f"[MANUAL BRIDGE] Listening on TCP port {TCP_PORT}")

        # Start accepting connections (blocking loop)
        self.accept_loop()

    # ========================================================
    # ACCEPT LOOP
    #
    # Waits for incoming TCP connections and handles them
    # sequentially.
    # ========================================================
    def accept_loop(self):

        while not rospy.is_shutdown():

            conn, addr = self.sock.accept()
            rospy.loginfo(f"[MANUAL BRIDGE] Laptop connected from {addr}")

            try:
                self.handle_connection(conn)

            except Exception as e:
                rospy.logwarn(f"[MANUAL BRIDGE] Connection error: {e}")

            finally:
                conn.close()
                rospy.loginfo("[MANUAL BRIDGE] Laptop disconnected")

    # ========================================================
    # CONNECTION HANDLER
    #
    # Reads raw TCP stream and reconstructs newline-delimited
    # command messages.
    # ========================================================
    def handle_connection(self, conn):

        buffer = ""

        while not rospy.is_shutdown():

            data = conn.recv(BUFFER_SIZE)

            # Connection closed by client
            if not data:
                break

            buffer += data.decode()

            # Process complete lines only
            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                line = line.strip()

                if line:
                    self.process_command(line)

    # ========================================================
    # COMMAND PARSER
    #
    # Expected format:
    #   CMD linear angular lift [tag auto state]
    #
    # Example:
    #   CMD 0.5 0.1 1 3 1 NAVIGATE
    # ========================================================
    def process_command(self, line):

        parts = line.split()

        # Ignore empty or malformed commands
        if not parts:
            return

        if parts[0] != "CMD":
            return

        # ----------------------------------------------------
        # REQUIRED FIELDS (velocity + actuator)
        # ----------------------------------------------------
        if len(parts) < 4:
            return

        try:
            linear  = float(parts[1])
            angular = float(parts[2])
            lift    = int(parts[3])
        except ValueError:
            return

        # ----------------------------------------------------
        # PUBLISH VELOCITY
        # ----------------------------------------------------
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.vel_pub.publish(twist)

        # ----------------------------------------------------
        # PUBLISH ACTUATOR COMMAND
        # ----------------------------------------------------
        self.act_pub.publish(Int32(lift))

        # ----------------------------------------------------
        # OPTIONAL EXTENDED FIELDS
        #
        # CMD linear angular lift tag auto state
        # ----------------------------------------------------
        if len(parts) >= 7:
            try:
                tag = int(parts[4])
                auto_state = bool(int(parts[5]))
                requested_state = parts[6]

                self.tag_pub.publish(Int32(tag))
                self.auto_pub.publish(Bool(auto_state))
                self.state_pub.publish(String(requested_state))

            except ValueError:
                # Ignore malformed optional fields
                pass


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":
    try:
        ManualControlBridge()
    except rospy.ROSInterruptException:
        pass
