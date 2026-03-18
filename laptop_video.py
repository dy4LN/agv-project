import gi
gi.require_version("Gst", "1.0")
gi.require_version("GstVideo", "1.0")

from gi.repository import Gst, GstVideo
from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import Qt

from config import VIDEO_UDP_PORT


class GStreamerWidget(QWidget):
    """
    QWidget wrapper for displaying a GStreamer H.264 UDP video stream.

    - Receives RTP H.264 packets over UDP
    - Decodes and renders using Direct3D sink (Windows)
    - Embeds video directly into Qt widget window
    """

    def __init__(self, status_callback, parent=None):
        super().__init__(parent)

        # Required for native window handle (used by GStreamer overlay)
        self.setAttribute(Qt.WA_NativeWindow)
        self.setAttribute(Qt.WA_PaintOnScreen)

        # GStreamer components
        self.pipeline = None
        self.video_sink = None

        # Callback for UI status updates
        self.status_callback = status_callback

    # =====================================================
    # PIPELINE CONTROL
    # =====================================================

    def start_pipeline(self):
        """
        Initialize and start the GStreamer pipeline.
        """
        # Ensure any previous pipeline is fully stopped
        self.stop_pipeline()

        # GStreamer pipeline description:
        # UDP → RTP → H264 depay → decode → convert → render
        desc = (
            f"udpsrc port={VIDEO_UDP_PORT} "
            "caps=application/x-rtp,media=video,encoding-name=H264,payload=96 "
            "! rtph264depay "
            "! avdec_h264 "
            "! videoconvert "
            "! d3dvideosink name=vsink"
        )

        # Create pipeline from description string
        self.pipeline = Gst.parse_launch(desc)

        # Get reference to the video sink (needed for window embedding)
        self.video_sink = self.pipeline.get_by_name("vsink")

        # Setup message bus for error handling and status updates
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self.on_bus_message)

        # Bind GStreamer video output to this Qt widget
        win_id = int(self.winId())
        if win_id and isinstance(self.video_sink, GstVideo.VideoOverlay):
            self.video_sink.set_window_handle(win_id)

        # Start pipeline
        ret = self.pipeline.set_state(Gst.State.PLAYING)

        if ret == Gst.StateChangeReturn.FAILURE:
            self.status_callback("Error: Failed to start stream")
        else:
            self.status_callback("Streaming...")

    def stop_pipeline(self):
        """
        Stop and clean up the GStreamer pipeline.
        """
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
            self.pipeline = None
            self.video_sink = None
            self.status_callback("Stopped")

    # =====================================================
    # GSTREAMER MESSAGE HANDLER
    # =====================================================

    def on_bus_message(self, bus, message):
        """
        Handle GStreamer bus messages (errors, EOS, etc.).
        """
        if message.type == Gst.MessageType.ERROR:
            err, _ = message.parse_error()
            self.status_callback(f"Error: {err.message}")
            self.stop_pipeline()

        elif message.type == Gst.MessageType.EOS:
            self.status_callback("Stream ended")
            self.stop_pipeline()
