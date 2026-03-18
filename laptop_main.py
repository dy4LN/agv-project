import sys
import gi

# Initialize GStreamer (required before use)
gi.require_version("Gst", "1.0")
from gi.repository import Gst

from PyQt5.QtWidgets import QApplication

from gui import AGVApp


def main():
    """
    Application entry point.

    - Initializes GStreamer (video pipeline backend)
    - Starts Qt application loop
    - Launches main AGV GUI
    """

    # Initialize GStreamer (must be done before creating pipelines)
    Gst.init(None)

    # Initialize Qt application
    app = QApplication(sys.argv)

    # Create and show main window
    window = AGVApp()
    window.show()

    # Start event loop
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
