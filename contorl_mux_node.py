#!/usr/bin/env python3

import rospy
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32


# =====================================================
# AGV VELOCITY MULTIPLEXER
# =====================================================
#
# Selects which controller (manual / align / nav)
# has authority based on mission_state.
#
# Includes watchdog timeout for safety.
# =====================================================


class AGVVelocityMux:

    def __init__(self):

        rospy.init_node("agv_velocity_mux")

        # =============================
        # INTERNAL STATE
        # =============================

        self.mission_state = "MANUAL"

        # Velocity inputs
        self.manual_vel = Twist()
        self.align_vel = Twist()
        self.nav_vel = Twist()

        # Actuator inputs
        self.manual_act = 0
        self.align_act = 0
        self.nav_act = 0

        # Watchdog timeout (seconds)
        self.timeout = 1.0

        # Last message timestamps
        self.manual_vel_time = 0
        self.align_vel_time = 0
        self.nav_vel_time = 0

        self.manual_act_time = 0
        self.align_act_time = 0
        self.nav_act_time = 0

        # =============================
        # SUBSCRIBERS
        # =============================

        rospy.Subscriber("mission_state", String, self.mission_cb)

        rospy.Subscriber("manual_cmd_vel", Twist, self.manual_vel_cb)
        rospy.Subscriber("align_cmd_vel", Twist, self.align_vel_cb)
        rospy.Subscriber("nav_cmd_vel", Twist, self.nav_vel_cb)

        rospy.Subscriber("manual_cmd_actuator", Int32, self.manual_act_cb)
        rospy.Subscriber("align_cmd_actuator", Int32, self.align_act_cb)
        rospy.Subscriber("nav_cmd_actuator", Int32, self.nav_act_cb)

        # =============================
        # PUBLISHERS
        # =============================

        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.cmd_act_pub = rospy.Publisher("cmd_actuator", Int32, queue_size=10)

        # =============================
        # MAIN LOOP
        # =============================

        rospy.Timer(rospy.Duration(0.02), self.update)  # 50 Hz

        rospy.loginfo("AGV Velocity Mux (Mission-Compatible) Running")
        rospy.spin()

    # =====================================================
    # CALLBACKS
    # =====================================================

    def mission_cb(self, msg):
        self.mission_state = msg.data

    # ---------- Velocity ----------

    def manual_vel_cb(self, msg):
        self.manual_vel = msg
        self.manual_vel_time = time.time()

    def align_vel_cb(self, msg):
        self.align_vel = msg
        self.align_vel_time = time.time()

    def nav_vel_cb(self, msg):
        self.nav_vel = msg
        self.nav_vel_time = time.time()

    # ---------- Actuator ----------

    def manual_act_cb(self, msg):
        self.manual_act = msg.data
        self.manual_act_time = time.time()

    def align_act_cb(self, msg):
        self.align_act = msg.data
        self.align_act_time = time.time()

    def nav_act_cb(self, msg):
        self.nav_act = msg.data
        self.nav_act_time = time.time()

    # =====================================================
    # WATCHDOG
    # =====================================================

    def fresh(self, timestamp):
        """Returns True if message is within timeout window."""
        return (time.time() - timestamp) < self.timeout

    # =====================================================
    # MAIN UPDATE LOOP
    # =====================================================

    def update(self, event):

        vel_out = Twist()
        act_out = 0

        state = self.mission_state

        # =============================
        # FSM-BASED AUTHORITY
        # =============================

        if state == "MANUAL":

            if self.fresh(self.manual_vel_time):
                vel_out = self.manual_vel

            if self.fresh(self.manual_act_time):
                act_out = self.manual_act

        elif state == "IDLE":
            # Safe stop while waiting
            vel_out = Twist()
            act_out = 0

        elif state == "PALLET_ALIGN":

            if self.fresh(self.align_vel_time):
                vel_out = self.align_vel

            if self.fresh(self.align_act_time):
                act_out = self.align_act

        elif state == "NAVIGATION":

            if self.fresh(self.nav_vel_time):
                vel_out = self.nav_vel

            if self.fresh(self.nav_act_time):
                act_out = self.nav_act

        elif state == "DONE":
            # Full stop after mission
            vel_out = Twist()
            act_out = 0

        else:
            # Unknown state → fail safe
            vel_out = Twist()
            act_out = 0

        # =============================
        # OUTPUT
        # =============================

        self.cmd_vel_pub.publish(vel_out)
        self.cmd_act_pub.publish(Int32(act_out))


# =====================================================
# ENTRY POINT
# =====================================================

if __name__ == "__main__":
    AGVVelocityMux()
