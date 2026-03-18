# =====================================================
# AGV APP CONFIGURATION
# =====================================================
# Centralized configuration for networking, controls,
# and motion constraints.
#
# NOTE:
# - Do NOT place logic here
# - This file should only contain constants
# =====================================================


# =====================================================
# NETWORK CONFIGURATION
# =====================================================

# Jetson TCP connection (command/control)
JETSON_IP   = "100.79.42.117"
JETSON_PORT = 8765

# Video streaming (UDP)
VIDEO_UDP_PORT = 5000

# Telemetry listener (UDP)
TELEMETRY_LISTEN_IP = "0.0.0.0"   # Listen on all interfaces
TELEMETRY_PORT      = 9000
TELEMETRY_STALE_SEC = 1.0         # Consider telemetry stale after this time


# =====================================================
# LOOP / TIMING CONFIGURATION
# =====================================================

HEARTBEAT_INTERVAL = 0.1   # Seconds between heartbeat messages to Jetson
LOOP_SLEEP         = 0.01  # Main loop sleep (controls CPU usage)


# =====================================================
# KEY MAPPINGS
# =====================================================

# Movement keys (WASD)
MOVE_KEYS = ['w', 'a', 's', 'd']

# Actuator control
LIFT_KEYS = ['j', 'k']   # j = down, k = up (based on your convention)

# Linear velocity adjustment
LINEAR_SPEED_KEYS = {
    "increase": 'i',
    "decrease": 'u'
}

# Angular velocity adjustment
ANGULAR_SPEED_KEYS = {
    "increase": 'p',
    "decrease": 'o'
}

# Priority order if multiple keys pressed
MOVE_PRIORITY = ['w', 's', 'a', 'd']


# =====================================================
# VELOCITY CONSTRAINTS
# =====================================================

# Linear velocity (m/s)
LINEAR_VEL_MIN   = 0.15
LINEAR_VEL_MAX   = 0.55
LINEAR_VEL_START = 0.30
LINEAR_VEL_STEP  = 0.05

# Angular velocity (rad/s)
ANGULAR_VEL_MIN   = 2.0
ANGULAR_VEL_MAX   = 8.0
ANGULAR_VEL_START = 4.0
ANGULAR_VEL_STEP  = 0.5
