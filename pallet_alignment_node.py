#!/usr/bin/env python3
"""
AGV Pallet Alignment FSM  (v3 -- pulse/settle only)
===================================================

States
------
PREP
    Lift forks up, then back down so the pallet tag becomes visible.

SEARCH
    Spin in short bursts with settle pauses until the desired AprilTag
    is detected.

CREEP
    Pulse / stop / settle loop for the full approach.
    This avoids caster-snap and spiral instability by never driving
    continuously.

    Each cycle:
        CHECK  -> read pose, compute angular correction, prepare pulse
        PULSE  -> drive forward for a short timed pulse
        SETTLE -> full stop, allow pose/filter to settle

    Pulse duration scales with distance:
        z > creep_far_z   -> creep_pulse_time_far
        z <= creep_far_z  -> creep_pulse_time_near

    Docks when z <= dock_distance.
    If tag is lost, robot stops and waits indefinitely.

DOCK
    Stop, raise forks for 3 s, publish /alignment_done = True.

Motor constraints (MotorInterfaceNode)
--------------------------------------
WHEEL_RADIUS = 0.0333375 m
WHEEL_BASE   = 0.127 m

MIN_RPM = 30   -> minimum linear speed ~ 0.105 m/s
               -> minimum spin speed   ~ 1.65 rad/s

MAX_RPM = 160  -> maximum linear speed ~ 0.559 m/s

Any |wheel RPM| < 30 is dead-banded up to 30.
Keep linear >= 0.11 m/s and pure-rotation >= 1.70 rad/s.
"""

import math
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_msgs.msg import Bool, Int32, String


# ============================================================
# TERMINAL COLOR / LOGGING HELPERS
# ============================================================

class C:
    RESET = "\033[0m"
    BOLD = "\033[1m"
    DIM = "\033[2m"
    RED = "\033[91m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    CYAN = "\033[96m"
    WHITE = "\033[97m"


def _banner(text, colour=C.CYAN):
    width = 58
    print(
        f"\n{colour}{C.BOLD}+{'=' * width}+"
        f"\n|  {text:<{width - 2}}|"
        f"\n+{'=' * width}+{C.RESET}"
    )


def _section(text, colour=C.YELLOW):
    print(f"{colour}{C.BOLD}>  {text}{C.RESET}")


def _ok(text):
    print(f"{C.GREEN}  [OK]  {text}{C.RESET}")


def _warn(text):
    rospy.logwarn(text)
    print(f"{C.YELLOW}  [!!]  {text}{C.RESET}")


def _err(text):
    rospy.logerr(text)
    print(f"{C.RED}  [XX]  {text}{C.RESET}")


def _dim(text):
    print(f"{C.DIM}      {text}{C.RESET}")


def _val(label, value, unit="", good=None):
    value_colour = C.GREEN if good is True else (C.RED if good is False else C.WHITE)
    print(f"      {C.DIM}{label:<26}{C.RESET}{value_colour}{value:>10.4f}  {unit}{C.RESET}")


# ============================================================
# PALLET ALIGNMENT FSM
# ============================================================

class PalletAlignFSM:

    DEBUG_PRINT_INTERVAL = 0.25  # seconds between console prints

    def __init__(self):
        rospy.init_node("agv_pallet_alignment")
        _banner("AGV Pallet Alignment FSM v3  --  starting up", C.CYAN)

        # --------------------------------------------------------
        # Mission state
        # --------------------------------------------------------
        self.active = False

        # --------------------------------------------------------
        # Sensor state
        # --------------------------------------------------------
        self.pose_msg = None
        self.visible = False
        self.detected_tag_id = None
        self.lost_counter = 0

        # --------------------------------------------------------
        # FSM state
        # --------------------------------------------------------
        self.state = "PREP"
        self.phase = "IDLE"
        self.phase_timer = rospy.Time(0)

        # --------------------------------------------------------
        # Debug throttle
        # --------------------------------------------------------
        self._last_debug = rospy.Time(0)

        # --------------------------------------------------------
        # Cached pulse command
        # --------------------------------------------------------
        self._cmd_linear = 0.0
        self._cmd_angular = 0.0

        # ========================================================
        # TUNING PARAMETERS
        # ========================================================

        # PREP
        self.prep_lift_up_time = 5.0    # s
        self.prep_lift_down_time = 4.0  # s

        # SEARCH
        self.search_spin_speed = 2.00   # rad/s (keep >= 1.70 for motor deadband)
        self.search_burst_time = 0.06   # s (~7 deg per burst)
        self.search_pause_time = 1.20   # s settle time between bursts

        # CREEP
        #
        # Short timed drive pulse -> full stop -> settle -> re-evaluate.
        # This avoids caster-snap and spiral instability.
        #
        # Controller each CHECK:
        #   angular = -(k_yaw * yaw + k_x * x), clamped to ±wz_max
        #   linear  = creep_speed
        #
        # Wheel check @ linear=0.11, wz_max=0.50:
        #   slow wheel = 0.11 - 0.50*0.0635 = 0.078 m/s -> 23 RPM
        #   which gets dead-banded up to 30 RPM = 0.105 m/s
        #   Small error is acceptable because each pulse is short and
        #   correction is recomputed every cycle from a fresh pose.
        #
        # Keep creep_speed >= 0.11 m/s.
        self.creep_speed = 0.11

        # Longer pulses when far, shorter pulses when near
        self.creep_far_z = 1.20              # m
        self.creep_pulse_time_far = 0.30     # s
        self.creep_pulse_time_near = 0.20    # s

        self.creep_settle_time = 0.35        # s

        # Angular correction gains
        self.creep_k_yaw = 0.80
        self.creep_k_x = 0.60
        self.creep_wz_max = 0.50

        # DOCK
        self.dock_distance = 0.50      # m
        self.lift_raise_time = 3.0     # s

        # ========================================================
        # ROS PUBLISHERS
        # ========================================================
        self.vel_pub = rospy.Publisher("/align_cmd_vel", Twist, queue_size=10)
        self.actuator_pub = rospy.Publisher("/align_cmd_actuator", Int32, queue_size=10)
        self.done_pub = rospy.Publisher("/alignment_done", Bool, queue_size=10)

        # ========================================================
        # ROS SUBSCRIBERS
        # ========================================================
        rospy.Subscriber("/mission_state", String, self.mission_state_cb)
        rospy.Subscriber("/pallet_pose_metric", PoseWithCovarianceStamped, self.pose_cb)
        rospy.Subscriber("/pallet_visible", Bool, self.visible_cb)
        rospy.Subscriber("/detected_tag_id", Int32, self.detected_tag_cb)

        # Main FSM loop (20 Hz)
        rospy.Timer(rospy.Duration(0.05), self.update)

        _ok("Node ready -- waiting for mission_state = PALLET_ALIGN")
        rospy.spin()

    # ============================================================
    # HELPERS
    # ============================================================

    def elapsed(self):
        return (rospy.Time.now() - self.phase_timer).to_sec()

    def _should_print(self):
        now = rospy.Time.now()
        if (now - self._last_debug).to_sec() >= self.DEBUG_PRINT_INTERVAL:
            self._last_debug = now
            return True
        return False

    def start_phase(self, phase):
        if self.phase != phase:
            _dim(f"phase  {self.phase}  ->  {phase}")
        self.phase = phase
        self.phase_timer = rospy.Time.now()

    def transition(self, new_state):
        """
        Transition to a new FSM state with a full stop first.
        """
        _banner(f"STATE  {self.state}  ->  {new_state}", C.CYAN)
        self.state = new_state
        self.phase = "IDLE"
        self.vel_pub.publish(Twist())

    def get_yaw(self):
        q = self.pose_msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def tag_ok(self):
        return self.visible and self.pose_msg is not None

    def send_vel(self, linear=0.0, angular=0.0):
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.vel_pub.publish(cmd)

    def clamp(self, value, low, high):
        return max(low, min(high, value))

    # ============================================================
    # CALLBACKS
    # ============================================================

    def mission_state_cb(self, msg):
        prev = self.active
        self.active = (msg.data == "PALLET_ALIGN")

        if not prev and self.active:
            _banner("MISSION ACTIVE  --  PALLET_ALIGN received", C.GREEN)
            self.reset()

        if prev and not self.active:
            _banner("MISSION INACTIVE  --  stopping FSM", C.RED)
            self.vel_pub.publish(Twist())
            self.reset()

    def reset(self):
        _section("FSM reset -- returning to PREP")
        self.pose_msg = None
        self.visible = False
        self.lost_counter = 0
        self.transition("PREP")
        self.start_phase("LIFT_UP")

    def pose_cb(self, msg):
        self.pose_msg = msg

    def visible_cb(self, msg):
        was_visible = self.visible
        self.visible = msg.data

        if self.visible and not was_visible:
            _ok(f"Tag VISIBLE  detected={self.detected_tag_id}")
        elif not self.visible and was_visible:
            _warn(f"Tag LOST  state={self.state}/{self.phase}")

        self.lost_counter = 0 if self.visible else self.lost_counter + 1

    def detected_tag_cb(self, msg):
        self.detected_tag_id = msg.data

    # ============================================================
    # MAIN LOOP (20 Hz)
    # ============================================================

    def update(self, event):
        if not self.active:
            return

        # --------------------------------------------------------
        # PREP
        # --------------------------------------------------------
        if self.state == "PREP":

            if self.phase == "LIFT_UP":
                self.actuator_pub.publish(Int32(1))

                if self._should_print():
                    _dim(
                        f"PREP / LIFT_UP  actuator=+1  "
                        f"{self.prep_lift_up_time - self.elapsed():.1f}s left"
                    )

                if self.elapsed() >= self.prep_lift_up_time:
                    _ok("PREP: lift UP done")
                    self.actuator_pub.publish(Int32(0))
                    self.start_phase("LIFT_DOWN")

            elif self.phase == "LIFT_DOWN":
                self.actuator_pub.publish(Int32(-1))

                if self._should_print():
                    _dim(
                        f"PREP / LIFT_DOWN  actuator=-1  "
                        f"{self.prep_lift_down_time - self.elapsed():.1f}s left"
                    )

                if self.elapsed() >= self.prep_lift_down_time:
                    _ok("PREP: lift DOWN done -- entering SEARCH")
                    self.actuator_pub.publish(Int32(0))
                    self.transition("SEARCH")
                    self.start_phase("BURST")

        # --------------------------------------------------------
        # SEARCH
        # --------------------------------------------------------
        elif self.state == "SEARCH":

            if self.tag_ok():
                _ok(f"SEARCH: tag {self.detected_tag_id} found -- entering CREEP")
                self.transition("CREEP")
                self.start_phase("CHECK")
                return

            if self.phase == "BURST":
                self.send_vel(angular=self.search_spin_speed)

                if self._should_print():
                    _dim(
                        f"SEARCH / BURST  spin={self.search_spin_speed:.2f} rad/s  "
                        f"{self.elapsed():.2f}/{self.search_burst_time:.2f}s  "
                        f"visible={self.visible}  det={self.detected_tag_id}"
                    )

                if self.elapsed() >= self.search_burst_time:
                    self.send_vel()
                    self.start_phase("PAUSE")

            elif self.phase == "PAUSE":

                if self._should_print():
                    _dim(
                        f"SEARCH / PAUSE  settling "
                        f"{self.search_pause_time - self.elapsed():.2f}s  "
                        f"visible={self.visible}  det={self.detected_tag_id}"
                    )

                if self.elapsed() >= self.search_pause_time:
                    self.start_phase("BURST")

        # --------------------------------------------------------
        # CREEP
        #
        # Pulse / stop / settle approach from detection to dock.
        # Never drives blind; if tag is lost, it stops and waits.
        # --------------------------------------------------------
        elif self.state == "CREEP":

            if not self.tag_ok():
                self.send_vel(0.0, 0.0)

                if self._should_print():
                    _warn(
                        f"CREEP: tag lost -- stopped, waiting to recover  "
                        f"lost={self.lost_counter} frames"
                    )
                return

            # ----------------------------------------------------
            # CHECK
            # ----------------------------------------------------
            if self.phase == "CHECK":

                yaw = self.get_yaw()
                x = self.pose_msg.pose.pose.position.x
                z = self.pose_msg.pose.pose.position.z

                yaw_ok = abs(yaw) < 0.035   # about 2 deg
                x_ok = abs(x) < 0.015       # 15 mm

                # Select pulse duration based on distance
                pulse_time = (
                    self.creep_pulse_time_far
                    if z > self.creep_far_z
                    else self.creep_pulse_time_near
                )

                if self._should_print():
                    _section(
                        f"CREEP / CHECK  z={z:.3f}m  pulse={pulse_time:.2f}s  "
                        f"lost={self.lost_counter}"
                    )
                    _val("yaw  (want 0)", math.degrees(yaw), "deg", good=yaw_ok)
                    _val("x    (want 0)", x, "m", good=x_ok)

                # Dock once close enough
                if z <= self.dock_distance:
                    _ok(f"CREEP: dock distance reached  z={z:.3f}m -- entering DOCK")
                    self.send_vel()
                    self.transition("DOCK")
                    self.start_phase("LIFT_UP")
                    return

                # Angular correction
                raw = -(self.creep_k_yaw * yaw + self.creep_k_x * x)
                angular = self.clamp(raw, -self.creep_wz_max, self.creep_wz_max)

                if self._should_print():
                    _val("-> cmd linear", self.creep_speed, "m/s")
                    _val("-> cmd angular", angular, "rad/s")

                self._cmd_linear = self.creep_speed
                self._cmd_angular = angular
                self._pulse_time = pulse_time
                self.start_phase("PULSE")

            # ----------------------------------------------------
            # PULSE
            # ----------------------------------------------------
            elif self.phase == "PULSE":
                self.send_vel(self._cmd_linear, self._cmd_angular)

                if self._should_print():
                    _dim(
                        f"CREEP / PULSE  lin={self._cmd_linear:.3f}  "
                        f"ang={self._cmd_angular:+.3f}  "
                        f"{self.elapsed():.2f}/{self._pulse_time:.2f}s"
                    )

                if self.elapsed() >= self._pulse_time:
                    self.send_vel()
                    self.start_phase("SETTLE")

            # ----------------------------------------------------
            # SETTLE
            # ----------------------------------------------------
            elif self.phase == "SETTLE":

                if self._should_print():
                    _dim(
                        f"CREEP / SETTLE  waiting "
                        f"{self.creep_settle_time - self.elapsed():.2f}s"
                    )

                if self.elapsed() >= self.creep_settle_time:
                    self.start_phase("CHECK")

        # --------------------------------------------------------
        # DOCK
        # --------------------------------------------------------
        elif self.state == "DOCK":

            if self.phase == "LIFT_UP":
                self.actuator_pub.publish(Int32(1))

                if self._should_print():
                    _dim(
                        f"DOCK / LIFT_UP  actuator=+1  "
                        f"{self.lift_raise_time - self.elapsed():.1f}s left"
                    )

                if self.elapsed() >= self.lift_raise_time:
                    self.actuator_pub.publish(Int32(0))
                    self.done_pub.publish(Bool(True))
                    _banner("PALLET PICK COMPLETE  --  /alignment_done = True", C.GREEN)
                    self.start_phase("DONE")

            elif self.phase == "DONE":
                pass  # idle until mission_state deactivates us


if __name__ == "__main__":
    PalletAlignFSM()
