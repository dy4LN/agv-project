import json
import socket
import time
from PyQt5.QtCore import QThread, pyqtSignal

from config import TELEMETRY_LISTEN_IP, TELEMETRY_PORT


class TelemetryReceiver(QThread):
    """
    UDP telemetry listener running in a background Qt thread.

    - Listens for JSON packets from the Jetson
    - Performs minimal validation
    - Emits parsed packets to the UI layer
    """

    # Emitted when a valid telemetry packet is received
    packet = pyqtSignal(dict)

    # Emitted for status updates (bind success, errors, etc.)
    status = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)

        self._running = False
        self._sock = None

    # =====================================================
    # CONTROL
    # =====================================================

    def start_receiver(self):
        """Start the telemetry receiver thread."""
        if self._running:
            return

        self._running = True
        self.start()

    def stop_receiver(self):
        """Stop the telemetry receiver thread and close socket."""
        self._running = False

        try:
            if self._sock:
                self._sock.close()
        except Exception:
            pass

        self._sock = None

    # =====================================================
    # MAIN THREAD LOOP
    # =====================================================

    def run(self):
        """Main thread execution: bind socket and receive packets."""
        try:
            # Create UDP socket
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

            # Bind to configured interface/port
            self._sock.bind((TELEMETRY_LISTEN_IP, TELEMETRY_PORT))

            # Timeout allows periodic loop checks for shutdown
            self._sock.settimeout(0.2)

            self.status.emit(f"Telemetry listening on UDP {TELEMETRY_PORT}")

        except Exception as e:
            self.status.emit(f"Telemetry bind error: {e}")
            self._running = False
            return

        # =========================
        # RECEIVE LOOP
        # =========================
        while self._running:
            try:
                data, addr = self._sock.recvfrom(65535)

            except socket.timeout:
                # Normal — allows thread to check _running flag
                continue

            except Exception as e:
                self.status.emit(f"Telemetry recv error: {e}")
                break

            # =========================
            # PARSE PACKET
            # =========================
            try:
                msg = json.loads(data.decode("utf-8", errors="ignore"))

                # Basic validation
                if not isinstance(msg, dict):
                    continue

                if msg.get("schema") != 1:
                    continue

                # Ensure timestamp exists
                if "ts" not in msg:
                    msg["ts"] = time.time()

                # Add source metadata (IP:port)
                msg["_src"] = f"{addr[0]}:{addr[1]}"

                # Debug option (disabled)
                # print("RX:", msg.get("seq"), time.time())

                # Emit to UI / consumer
                self.packet.emit(msg)

            except Exception:
                # Ignore malformed JSON packets
                continue

        # =========================
        # CLEANUP
        # =========================
        try:
            if self._sock:
                self._sock.close()
        except Exception:
            pass

        self._sock = None
