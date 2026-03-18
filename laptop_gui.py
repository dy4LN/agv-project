from PyQt5.QtWidgets import (
    QWidget, QPushButton, QVBoxLayout,
    QLabel, QHBoxLayout, QComboBox
)
from PyQt5.QtCore import Qt, QTimer

from video import GStreamerWidget
from manual_control import ManualController
from telemetry_receiver import TelemetryReceiver
from telemetry_widgets import TelemetryDashboard


class AGVApp(QWidget):
    """
    Main GUI application for AGV control.

    Combines:
    - Video streaming
    - Manual control interface
    - Telemetry visualization
    - Mode/state management
    """

    def __init__(self):
        super().__init__()

        self.setWindowTitle("AGV Control")
        self.setGeometry(100, 100, 1000, 720)

        # ==================================================
        # INTERNAL STATE
        # ==================================================
        self.is_auto = False
        self.stream_on = False

        # ==================================================
        # STATUS LABELS
        # ==================================================
        self.status_label = QLabel("Idle")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet(
            "color: white; background-color: gray; padding: 6px;"
        )

        self.mode_status_label = QLabel("Mode: Manual")
        self.mode_status_label.setAlignment(Qt.AlignCenter)
        self.mode_status_label.setStyleSheet(
            "color: white; background-color: darkred; padding: 6px;"
        )

        # ==================================================
        # VIDEO STREAM
        # ==================================================
        self.video_widget = GStreamerWidget(self.update_status)
        self.video_widget.setMinimumSize(640, 480)

        # ==================================================
        # MANUAL CONTROLLER
        # ==================================================
        self.manual = ManualController(self.update_manual_status)

        # ==================================================
        # TELEMETRY SYSTEM
        # ==================================================
        self.telemetry_dashboard = TelemetryDashboard()

        self.telemetry_rx = TelemetryReceiver()
        self.telemetry_rx.packet.connect(
            self.telemetry_dashboard.update_from_packet
        )
        self.telemetry_rx.status.connect(self.update_status)
        self.telemetry_rx.start_receiver()

        # Timer to detect stale telemetry
        self.stale_timer = QTimer(self)
        self.stale_timer.timeout.connect(
            self.telemetry_dashboard.set_stale_if_needed
        )
        self.stale_timer.start(200)

        # ==================================================
        # CONTROLS
        # ==================================================

        # Stream toggle
        self.stream_button = QPushButton("Stream: OFF")
        self.stream_button.clicked.connect(self.toggle_stream)

        # Mode toggle (manual vs auto)
        self.mode_button = QPushButton("Manual Control")
        self.mode_button.clicked.connect(self.toggle_mode)

        # Mission state selector
        self.state_dropdown = QComboBox()
        self.state_dropdown.addItems(
            ["IDLE", "PALLET_ALIGN", "NAVIGATE", "DONE"]
        )
        self.state_dropdown.currentTextChanged.connect(
            self.update_requested_state
        )

        # Destination tag selector
        self.tag_dropdown = QComboBox()
        self.tag_dropdown.addItems(
            [str(i) for i in range(0, 11)]
        )
        self.tag_dropdown.currentTextChanged.connect(
            self.update_tag
        )

        # ==================================================
        # INITIAL MODE SETUP (MANUAL DEFAULT)
        # ==================================================
        self.apply_manual_mode_ui()

        # ==================================================
        # LAYOUTS
        # ==================================================

        # Control bar
        control_layout = QHBoxLayout()
        control_layout.addWidget(self.stream_button)
        control_layout.addWidget(self.mode_button)
        control_layout.addWidget(self.state_dropdown)
        control_layout.addWidget(self.tag_dropdown)

        # Top section: video + telemetry
        top_layout = QHBoxLayout()
        top_layout.addWidget(self.video_widget, stretch=1)
        top_layout.addWidget(self.telemetry_dashboard, stretch=1)

        # Main layout
        layout = QVBoxLayout()
        layout.addLayout(top_layout)
        layout.addLayout(control_layout)
        layout.addWidget(self.status_label)
        layout.addWidget(self.mode_status_label)

        self.setLayout(layout)

    # ==================================================
    # STREAM CONTROL
    # ==================================================

    def toggle_stream(self):
        """Toggle video streaming on/off."""
        self.stream_on = not self.stream_on

        if self.stream_on:
            self.video_widget.start_pipeline()
            self.stream_button.setText("Stream: ON")
        else:
            self.video_widget.stop_pipeline()
            self.stream_button.setText("Stream: OFF")

    # ==================================================
    # MODE CONTROL
    # ==================================================

    def toggle_mode(self):
        """Switch between manual and autonomous modes."""
        self.is_auto = not self.is_auto

        if self.is_auto:
            self.apply_auto_mode_ui()
            self.manual.set_auto_state(1)
            self.manual.stop()
        else:
            self.apply_manual_mode_ui()
            self.manual.set_auto_state(0)
            self.manual.start()

    # ==================================================
    # UI MODE HELPERS
    # ==================================================

    def apply_manual_mode_ui(self):
        """Configure UI for manual control mode."""
        self.mode_button.setText("Manual Control")
        self.mode_status_label.setText("Mode: Manual")
        self.mode_status_label.setStyleSheet(
            "color: white; background-color: darkred; padding: 6px;"
        )

        # Enable tag selection
        self.tag_dropdown.setEnabled(True)
        self.tag_dropdown.setStyleSheet("")

        # Disable mission state selection
        self.state_dropdown.setEnabled(False)
        self.state_dropdown.setStyleSheet(
            "QComboBox:disabled { background-color: #444; color: #888; }"
        )

    def apply_auto_mode_ui(self):
        """Configure UI for autonomous mode."""
        self.mode_button.setText("Autonomous Control")
        self.mode_status_label.setText("Mode: Autonomous")
        self.mode_status_label.setStyleSheet(
            "color: white; background-color: green; padding: 6px;"
        )

        # Enable mission state selection
        self.state_dropdown.setEnabled(True)
        self.state_dropdown.setStyleSheet("")

        # Disable tag selection
        self.tag_dropdown.setEnabled(False)
        self.tag_dropdown.setStyleSheet(
            "QComboBox:disabled { background-color: #444; color: #888; }"
        )

    # ==================================================
    # DROPDOWN HANDLERS
    # ==================================================

    def update_requested_state(self, state):
        """Send selected mission state to controller."""
        self.manual.set_requested_state(state)

    def update_tag(self, tag):
        """Send selected destination tag."""
        self.manual.set_desired_tag(int(tag))

    # ==================================================
    # STATUS UPDATES
    # ==================================================

    def update_status(self, text):
        """Update general status label."""
        self.status_label.setText(text)

    def update_manual_status(self, active):
        """Update manual mode status indicator."""
        if not self.is_auto:
            if active:
                self.mode_status_label.setText("Mode: Manual (Active)")
                self.mode_status_label.setStyleSheet(
                    "color: white; background-color: green; padding: 6px;"
                )

    # ==================================================
    # CLEAN SHUTDOWN
    # ==================================================

    def closeEvent(self, event):
        """Ensure all subsystems shut down cleanly."""
        self.video_widget.stop_pipeline()
        self.manual.stop()
        self.telemetry_rx.stop_receiver()
        super().closeEvent(event)
