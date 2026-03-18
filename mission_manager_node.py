#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import Point


class AGVMissionManager:
    """
    High-level mission state manager.

    Responsibilities:
    - Handles FSM transitions (MANUAL, IDLE, PALLET_ALIGN, NAVIGATION, DONE)
    - Maps tag IDs to GPS delivery locations
    - Validates state transitions based on system readiness
    """

    def __init__(self):

        rospy.init_node("agv_mission_manager")

        # ============================================================
        # LOAD TAG → GPS MAP (from ROS parameter server)
        # ============================================================

        self.tag_map = rospy.get_param("tag_locations", {})

        # Normalize keys (int) and values (float)
        normalized = {}
        for k, v in self.tag_map.items():
            try:
                tag_id = int(k)
                if isinstance(v, (list, tuple)) and len(v) >= 2:
                    normalized[tag_id] = [float(v[0]), float(v[1])]
            except Exception:
                continue

        self.tag_map = normalized
        rospy.loginfo(f"Loaded {len(self.tag_map)} tag locations")

        # ============================================================
        # ROS PUBLISHERS
        # ============================================================

        self.mission_state_pub = rospy.Publisher("/mission_state", String, queue_size=10)
        self.tag_id_pub = rospy.Publisher("/desired_tag_id", Int32, queue_size=10)
        self.delivery_location_pub = rospy.Publisher("/delivery_location", Point, queue_size=10)

        # ============================================================
        # ROS SUBSCRIBERS
        # ============================================================

        rospy.Subscriber("/alignment_done", Bool, self.alignment_done_callback)
        rospy.Subscriber("/delivery_done", Bool, self.delivery_done_callback)
        rospy.Subscriber("/auto_state", Bool, self.auto_state_callback)
        rospy.Subscriber("/requested_state", String, self.requested_state_callback)
        rospy.Subscriber("/input_tag_id", Int32, self.input_tag_id_callback)
        rospy.Subscriber(
            "/delivery_location_received",
            Bool,
            self.delivery_location_received_callback
        )

        # ============================================================
        # FSM STATE VARIABLES
        # ============================================================

        self.state = "MANUAL"

        self.alignment_done = False
        self.delivery_done = False
        self.auto_state = False

        self.desired_tag_id = None
        self.delivery_location = Point()
        self.delivery_location_received = False

        self.requested_state = None
        self.last_processed_request = None

        # Main FSM update loop (10 Hz)
        rospy.Timer(rospy.Duration(0.1), self.update)

        rospy.loginfo("AGV Mission Manager Ready")

    # ============================================================
    # CALLBACKS (Input Signals)
    # ============================================================

    def alignment_done_callback(self, msg):
        self.alignment_done = msg.data

    def delivery_done_callback(self, msg):
        self.delivery_done = msg.data

    def auto_state_callback(self, msg):
        self.auto_state = msg.data

    def requested_state_callback(self, msg):
        self.requested_state = msg.data

    def input_tag_id_callback(self, msg):
        """
        Converts selected tag ID into a GPS delivery location.
        """
        self.desired_tag_id = msg.data

        if self.desired_tag_id in self.tag_map:
            lat, lon = self.tag_map[self.desired_tag_id]

            self.delivery_location.x = lat
            self.delivery_location.y = lon
            self.delivery_location.z = 0.0

            self.delivery_location_received = True

            rospy.loginfo(
                f"Tag {self.desired_tag_id} → GPS ({lat}, {lon})"
            )
        else:
            rospy.logwarn(
                f"Tag {self.desired_tag_id} not found in tag_locations"
            )
            self.delivery_location_received = False

    def delivery_location_received_callback(self, msg):
        self.delivery_location_received = msg.data

    # ============================================================
    # TRANSITION VALIDATION
    #
    # Ensures state transitions only occur when system conditions
    # are satisfied (prevents invalid or premature transitions).
    # ============================================================

    def is_valid_transition(self, new_state):

        if new_state == "MANUAL":
            return True

        if new_state == "IDLE":
            return True

        if new_state == "PALLET_ALIGN":
            return (
                self.delivery_location_received and
                self.desired_tag_id is not None
            )

        if new_state == "NAVIGATION":
            return self.delivery_location_received

        if new_state == "DONE":
            return self.delivery_done

        return False

    # ============================================================
    # FSM UPDATE LOOP
    #
    # Handles:
    # - Manual override
    # - External state requests
    # - Internal automatic transitions
    # ============================================================

    def update(self, event):

        previous_state = self.state

        # --------------------------------------------------------
        # Manual override (highest priority)
        # --------------------------------------------------------
        if not self.auto_state:
            self.state = "MANUAL"

        else:

            # Transition from MANUAL → IDLE when auto enabled
            if self.state == "MANUAL":
                self.state = "IDLE"

            # ----------------------------------------------------
            # Process requested state (only if changed)
            # ----------------------------------------------------
            if (
                self.requested_state is not None and
                self.requested_state != self.last_processed_request
            ):

                if self.is_valid_transition(self.requested_state):
                    rospy.loginfo(f"Accepted state: {self.requested_state}")
                    self.state = self.requested_state
                    self.last_processed_request = self.requested_state
                else:
                    rospy.logwarn(f"Rejected invalid state: {self.requested_state}")

            # ----------------------------------------------------
            # Internal FSM transitions
            # ----------------------------------------------------
            else:

                if self.state == "PALLET_ALIGN":

                    if self.alignment_done:
                        self.state = "NAVIGATION"
                        self.alignment_done = False  # reset latch

                elif self.state == "NAVIGATION":

                    if self.delivery_done:
                        self.state = "DONE"
                        self.delivery_done = False  # reset latch

        # Log state transitions
        if previous_state != self.state:
            rospy.loginfo(f"Mission State: {previous_state} → {self.state}")

        self.publish()

    # ============================================================
    # PUBLISH CURRENT STATE
    # ============================================================

    def publish(self):

        self.mission_state_pub.publish(String(self.state))

        self.tag_id_pub.publish(
            Int32(self.desired_tag_id if self.desired_tag_id is not None else -1)
        )

        self.delivery_location_pub.publish(self.delivery_location)


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":
    try:
        AGVMissionManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
