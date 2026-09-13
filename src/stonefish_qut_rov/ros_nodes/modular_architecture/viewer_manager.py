"""Optional viewer processes; control logic never waits for viewer shutdown."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import sys

from ament_index_python.packages import get_package_prefix

from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, DurabilityPolicy

from process_utils import start_process, stop_process
from rov_config import CAMERA_SHOW_TOPIC, FISH_FOLLOW_EXECUTABLE, PLOTTER_EXECUTABLE, REAL_CAMERA_EXECUTABLE


class ViewerManager:
    def __init__(self, node, package, mode):
        self.node, self.package, self.mode = node, package, mode
        self.camera = None
        self.plotter = None
        self.visible = False
        self.retired = ThreadPoolExecutor(max_workers=2)
        self.show_pub = node.create_publisher(
            Bool, CAMERA_SHOW_TOPIC,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    @staticmethod
    def running(process):
        return process is not None and process.poll() is None

    def _start(self, executable):
        try:
            path = Path(get_package_prefix(self.package)) / "lib" / self.package / executable
            # Direct Python child: stop it individually, but keep it in teleop's
            # process group so the stack supervisor also cleans up orphan viewers.
            return start_process([sys.executable, str(path)], new_session=False)
        except (OSError, LookupError) as error:
            self.node.get_logger().error(f"Cannot start {executable}: {error}")
            return None

    def start_camera(self):
        if not self.running(self.camera):
            self._retire(self.camera)
            executable = FISH_FOLLOW_EXECUTABLE if self.mode == "sim" else REAL_CAMERA_EXECUTABLE
            self.camera = self._start(executable)

    def _retire(self, process):
        if process is not None:
            self.retired.submit(stop_process, process, process_group=False)

    def toggle_camera(self):
        if self.mode == "sim" and self.running(self.camera):
            self.close_camera()
        else:
            self.start_camera()
            if self.mode != "sim":
                self.visible = not self.visible
                msg = Bool()
                msg.data = self.visible
                self.show_pub.publish(msg)

    def close_camera(self):
        self._retire(self.camera)
        self.camera = None

    def toggle_plotter(self):
        if self.running(self.plotter):
            self._retire(self.plotter)
            self.plotter = None
        else:
            self._retire(self.plotter)
            self.plotter = self._start(PLOTTER_EXECUTABLE)

    def close(self):
        self.close_camera()
        self._retire(self.plotter)
        self.plotter = None
        self.retired.shutdown(wait=True)
