import time
import numpy as np
import pyqtgraph as pg

from PyQt5.QtWidgets import (
    QWidget, QLabel, QGridLayout,
    QProgressBar, QVBoxLayout, QHBoxLayout,
    QSizePolicy, QFrame
)
from PyQt5.QtCore import Qt, QEvent

from config import TELEMETRY_STALE_SEC


# ============================================================
# HELPER UTILITIES
# ============================================================

def _get(d, path, default=None):
    """
    Safe nested dictionary access.

    Example:
        _get(pkt, ["motors", "left", "rpm"])
    """
    cur = d
    for k in path:
        if not isinstance(cur, dict) or k not in cur:
            return default
        cur = cur[k]
    return cur


# ============================================================
# TELEMETRY SUMMARY PANEL
# ============================================================

class TelemetryPanel(QWidget):
    """
    Displays summarized telemetry information:
    - State, fault status
    - Speeds and RPMs
    - PWM and targets
    - Peripheral stats (IMU, encoders)
    """

    def __init__(self, parent=None):
        super().__init__(parent)

        self._last_rx_time = 0.0

        # ---------- Header ----------
        self.header = QLabel("Telemetry: Waiting…")
        self.header.setAlignment(Qt.AlignCenter)
        self.header.setStyleSheet("color: white; background-color: #444; padding: 6px;")

        # ---------- Core status ----------
        self.state = QLabel("state: —")
        self.fault = QLabel("fault: —")
        self.age = QLabel("age: — ms")

        # ---------- Motion ----------
        self.speed = QLabel("speed: — m/s, — rad/s")
        self.motors = QLabel("motors: L — rpm | R — rpm")

        # ---------- Command vs actual ----------
        self.cmd = QLabel("cmd: — m/s, — rad/s")
        self.actual = QLabel("actual: — m/s, — rad/s")

        # ---------- Motor + actuator ----------
        self.pwm = QLabel("pwm: L — | R —")
        self.target = QLabel("target rpm: L — | R —")
        self.actuator = QLabel("actuator: —")

        # ---------- Peripherals ----------
        self.periph = QLabel("peripherals: imu — hz | enc L — | enc R —")
        self.teensy_ts = QLabel("teensy ts: — us")

        # ---------- Layout ----------
        grid = QGridLayout()
        grid.addWidget(self.header, 0, 0, 1, 2)

        grid.addWidget(self.state, 1, 0)
        grid.addWidget(self.fault, 1, 1)
        grid.addWidget(self.age, 2, 0, 1, 2)

        grid.addWidget(self.speed, 3, 0, 1, 2)
        grid.addWidget(self.motors, 4, 0, 1, 2)

        grid.addWidget(self.cmd, 5, 0, 1, 2)
        grid.addWidget(self.actual, 6, 0, 1, 2)

        grid.addWidget(self.pwm, 7, 0, 1, 2)
        grid.addWidget(self.target, 8, 0, 1, 2)
        grid.addWidget(self.actuator, 9, 0, 1, 2)

        grid.addWidget(self.periph, 10, 0, 1, 2)
        grid.addWidget(self.teensy_ts, 11, 0, 1, 2)

        self.setLayout(grid)

    def update_from_packet(self, pkt: dict):
        """
        Update all UI fields from a telemetry packet.
        """
        now = time.time()
        self._last_rx_time = now

        ts = float(pkt.get("ts", now))
        age_ms = max(0.0, (now - ts) * 1000.0)

        # ---------- Extract values ----------
        state = pkt.get("state", "—")
        fault = pkt.get("fault", "—")

        lin = _get(pkt, ["speed", "linear_mps"])
        ang = _get(pkt, ["speed", "angular_rps"])

        l_rpm = _get(pkt, ["motors", "left", "rpm"])
        r_rpm = _get(pkt, ["motors", "right", "rpm"])

        targetL = _get(pkt, ["motors", "left", "target_rpm"])
        targetR = _get(pkt, ["motors", "right", "target_rpm"])

        pwmL = _get(pkt, ["motors", "left", "pwm"])
        pwmR = _get(pkt, ["motors", "right", "pwm"])

        imu_hz = _get(pkt, ["peripherals", "imu_hz"])
        enc_hz_L = _get(pkt, ["motors", "left", "enc_hz"])
        enc_hz_R = _get(pkt, ["motors", "right", "enc_hz"])

        actuator_state = _get(pkt, ["actuator", "state"])
        teensy_ts = _get(pkt, ["debug", "teensy_timestamp_us"])

        # ---------- Actuator mapping ----------
        if actuator_state == 1:
            actuator_text = "EXTEND"
        elif actuator_state == -1:
            actuator_text = "RETRACT"
        elif actuator_state == 0:
            actuator_text = "OFF"
        else:
            actuator_text = "—"

        # ---------- Header state ----------
        ok = (fault == "NONE" or fault == 0 or fault is None)

        if ok:
            self.header.setText("Telemetry: OK")
            self.header.setStyleSheet("color: white; background-color: #2b7; padding: 6px;")
        else:
            self.header.setText("Telemetry: FAULT")
            self.header.setStyleSheet("color: white; background-color: #b22; padding: 6px;")

        # ---------- Update labels ----------
        self.state.setText(f"state: {state}")
        self.fault.setText(f"fault: {fault}")
        self.age.setText(f"age: {age_ms:.0f} ms")

        self.speed.setText(
            f"speed: {lin:.3f} m/s, {ang:.3f} rad/s"
            if lin is not None else "speed: —"
        )

        self.motors.setText(
            f"motors: L {l_rpm:.1f} rpm | R {r_rpm:.1f} rpm"
            if l_rpm is not None else "motors: —"
        )

        self.pwm.setText(f"pwm: L {pwmL} | R {pwmR}")
        self.target.setText(f"target rpm: L {targetL} | R {targetR}")
        self.actuator.setText(f"actuator: {actuator_text}")

        self.periph.setText(
            f"peripherals: imu {imu_hz} hz | enc L {enc_hz_L} | enc R {enc_hz_R}"
        )

        self.teensy_ts.setText(f"teensy ts: {teensy_ts} us")

    def set_stale_if_needed(self):
        """
        Mark telemetry as stale if no packets received recently.
        """
        if self._last_rx_time == 0.0:
            return

        if (time.time() - self._last_rx_time) > TELEMETRY_STALE_SEC:
            self.header.setText("Telemetry: STALE")
            self.header.setStyleSheet("color: white; background-color: #b80; padding: 6px;")


# ============================================================
# LIVE SPEED PLOT
# ============================================================

class LiveSpeedPlot(QWidget):
    """
    Real-time plot of commanded vs actual linear velocity.
    """

    def __init__(self, window_sec=10.0, hz=20.0, parent=None):
        super().__init__(parent)

        self.n = int(window_sec * hz)

        self.x = np.linspace(-window_sec, 0.0, self.n)
        self.cmd = np.zeros(self.n)
        self.act = np.zeros(self.n)

        self.plot = pg.PlotWidget(title="Linear Speed (last 10 s)")
        self.plot.showGrid(x=True, y=True)
        self.plot.setLabel("left", "m/s")
        self.plot.setLabel("bottom", "time (s)")
        self.plot.setYRange(-1.0, 1.0, padding=0)
        self.plot.setXRange(-window_sec, 0.0, padding=0)
        self.plot.addLegend()

        self.cmd_curve = self.plot.plot(self.x, self.cmd, pen=pg.mkPen("y", width=2), name="cmd")
        self.act_curve = self.plot.plot(self.x, self.act, pen=pg.mkPen("c", width=2), name="actual")

        layout = QVBoxLayout()
        layout.addWidget(self.plot)
        self.setLayout(layout)

    def update_from_packet(self, pkt: dict):
        try:
            cmd = _get(pkt, ["motion", "cmd", "linear_mps"], 0.0)
            act = _get(pkt, ["motion", "actual", "linear_mps"], 0.0)
        except Exception:
            return

        self.cmd[:-1] = self.cmd[1:]
        self.act[:-1] = self.act[1:]

        self.cmd[-1] = cmd or 0.0
        self.act[-1] = act or 0.0

        self.cmd_curve.setData(self.x, self.cmd)
        self.act_curve.setData(self.x, self.act)
