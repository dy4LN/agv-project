import socket
import threading
import time
import keyboard

from config import (
    JETSON_IP, JETSON_PORT,
    MOVE_PRIORITY,
    LINEAR_SPEED_KEYS, ANGULAR_SPEED_KEYS,
    LINEAR_VEL_MIN, LINEAR_VEL_MAX, LINEAR_VEL_START, LINEAR_VEL_STEP,
    ANGULAR_VEL_MIN, ANGULAR_VEL_MAX, ANGULAR_VEL_START, ANGULAR_VEL_STEP,
    HEARTBEAT_INTERVAL, LOOP_SLEEP
)


class ManualController:
    """
    Handles manual control input and communication with the Jetson.

    Responsibilities:
    - Capture keyboard input
    - Manage velocity state (linear/angular)
    - Send control packets over TCP
    - Handle manual vs auto mode behavior
    """

    def __init__(self, status_callback):
        # Networking
        self.sock = None
        self.running = False
        self.thread = None

        # UI callback
        self.status_callback = status_callback

        # ==================================================
        # Velocity configuration
        # ==================================================
        self.linear_speed = LINEAR_VEL_START
        self.angular_speed = ANGULAR_VEL_START

        # ==================================================
        # Command state (sent to Jetson)
        # ==================================================
        self.linear_cmd = 0.0
        self.angular_cmd = 0.0
        self.lift_state = 0

        # ==================================================
        # Mission / GUI state
        # ==================================================
        self.auto_state = 0
        self.requested_state = "Idle"
        self.desired_tag = 1

        # Last packet (used to avoid unnecessary sends)
        self.last_packet = ""

    # ==================================================
    # GUI SETTERS
    # ==================================================

    def set_auto_state(self, value):
        self.auto_state = int(value)

    def set_requested_state(self, state):
        self.requested_state = str(state)

    def set_desired_tag(self, tag):
        self.desired_tag = int(tag)

    # ==================================================
    # NETWORK UTILITIES
    # ==================================================

    def safe_send(self, msg: str) -> bool:
        """
        Safely send a TCP message.
        Returns True on success, False on failure.
        """
        try:
            if self.sock:
                self.sock.sendall(msg.encode())
                return True
        except Exception:
            pass
        return False

    # ==================================================
    # UTILS
    # ==================================================

    def clamp(self, value, min_v, max_v):
        """Clamp value within bounds."""
        return max(min(value, max_v), min_v)

    # ==================================================
    # START / STOP
    # ==================================================

    def start(self):
        """Initialize TCP connection and start control loop."""
        if self.running:
            return

        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((JETSON_IP, JETSON_PORT))

            self.running = True

            # Send initial packet (safe startup state)
            self.send_packet()

            # Start background thread
            self.thread = threading.Thread(
                target=self.loop, daemon=True
            )
            self.thread.start()

            self.status_callback(True)

        except Exception:
            self.stop()

    def stop(self):
        """Stop controller and send safe zero command."""
        self.running = False

        if self.sock:
            # Safety stop
            self.linear_cmd = 0.0
            self.angular_cmd = 0.0
            self.lift_state = 0

            self.send_packet()

            self.sock.close()
            self.sock = None

        self.status_callback(False)

    # ==================================================
    # INPUT HANDLING
    # ==================================================

    def pick_active_move_key(self):
        """
        Resolve which movement key is active based on priority.
        Prevents conflicting commands (e.g., W + S).
        """
        for k in MOVE_PRIORITY:
            if keyboard.is_pressed(k):
                return k
        return None

    # ==================================================
    # PACKET BUILDING
    # ==================================================

    def build_packet(self):
        """
        Construct command packet sent to Jetson.

        Format:
        CMD linear angular lift tag auto_state requested_state
        """
        return (
            f"CMD "
            f"{self.linear_cmd:.2f} "
            f"{self.angular_cmd:.2f} "
            f"{self.lift_state} "
            f"{self.desired_tag} "
            f"{self.auto_state} "
            f"{self.requested_state}\n"
        )

    def send_packet(self):
        """
        Send packet, avoiding redundant transmissions when unchanged.
        """
        packet = self.build_packet()

        if packet != self.last_packet:
            self.safe_send(packet)
            self.last_packet = packet
        else:
            # Still send for heartbeat consistency
            self.safe_send(packet)

    # ==================================================
    # MAIN LOOP
    # ==================================================

    def loop(self):
        """Main control loop (runs in background thread)."""
        last_hb = time.time()

        while self.running:

            # ==================================================
            # MANUAL MODE CONTROL
            # ==================================================
            if self.auto_state == 0:

                # ---------- Linear speed adjustment ----------
                if keyboard.is_pressed(LINEAR_SPEED_KEYS["increase"]):
                    self.linear_speed += LINEAR_VEL_STEP
                    self.linear_speed = self.clamp(
                        self.linear_speed,
                        LINEAR_VEL_MIN,
                        LINEAR_VEL_MAX
                    )
                    time.sleep(0.2)

                if keyboard.is_pressed(LINEAR_SPEED_KEYS["decrease"]):
                    self.linear_speed -= LINEAR_VEL_STEP
                    self.linear_speed = self.clamp(
                        self.linear_speed,
                        LINEAR_VEL_MIN,
                        LINEAR_VEL_MAX
                    )
                    time.sleep(0.2)

                # ---------- Angular speed adjustment ----------
                if keyboard.is_pressed(ANGULAR_SPEED_KEYS["increase"]):
                    self.angular_speed += ANGULAR_VEL_STEP
                    self.angular_speed = self.clamp(
                        self.angular_speed,
                        ANGULAR_VEL_MIN,
                        ANGULAR_VEL_MAX
                    )
                    time.sleep(0.2)

                if keyboard.is_pressed(ANGULAR_SPEED_KEYS["decrease"]):
                    self.angular_speed -= ANGULAR_VEL_STEP
                    self.angular_speed = self.clamp(
                        self.angular_speed,
                        ANGULAR_VEL_MIN,
                        ANGULAR_VEL_MAX
                    )
                    time.sleep(0.2)

                # ---------- Movement ----------
                linear_cmd = 0.0
                angular_cmd = 0.0

                active_move = self.pick_active_move_key()

                if active_move == 'w':
                    linear_cmd = self.linear_speed
                elif active_move == 's':
                    linear_cmd = -self.linear_speed
                elif active_move == 'a':
                    angular_cmd = self.angular_speed
                elif active_move == 'd':
                    angular_cmd = -self.angular_speed

                # ---------- Lift control ----------
                if keyboard.is_pressed('k'):
                    lift_state = 1
                elif keyboard.is_pressed('j'):
                    lift_state = -1
                else:
                    lift_state = 0

                self.linear_cmd = linear_cmd
                self.angular_cmd = angular_cmd
                self.lift_state = lift_state

            # ==================================================
            # AUTO MODE
            # ==================================================
            else:
                # Disable manual motion in auto
                self.linear_cmd = 0.0
                self.angular_cmd = 0.0
                self.lift_state = 0

            # ==================================================
            # SEND COMMAND
            # ==================================================
            self.send_packet()

            # ==================================================
            # HEARTBEAT
            # ==================================================
            if time.time() - last_hb >= HEARTBEAT_INTERVAL:
                self.safe_send(self.build_packet())
                last_hb = time.time()

            time.sleep(LOOP_SLEEP)

        # ==================================================
        # FINAL SAFETY STOP
        # ==================================================
        self.linear_cmd = 0.0
        self.angular_cmd = 0.0
        self.lift_state = 0
        self.safe_send(self.build_packet())
