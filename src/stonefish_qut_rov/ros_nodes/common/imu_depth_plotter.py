#!/usr/bin/env python3
"""
Live IMU + depth plotter.

Subscribes to the COMMON topics, so it works unchanged in both sim and
real mode:
    /qut_rov/depth   (std_msgs/Float64, metres, positive down)
    /qut_rov/imu     (sensor_msgs/Imu)

Displays a rolling window:
    left  — depth vs time (y axis inverted, deeper is lower)
    right — pitch, roll, yaw stacked vertically on independent axes

Launched and killed by teleop_controller via the D-pad up toggle, but
can also be run standalone:
    ros2 run stonefish_qut_rov imu_depth_plotter.py
"""

import math
import signal
import threading
from collections import deque

import matplotlib.pyplot as plt
import rclpy
from matplotlib.animation import FuncAnimation
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

from control_utils import quaternion_to_rpy
from rov_config import (
    DEPTH_TOPIC,
    IMU_TOPIC,
    PLOT_REFRESH_MS,
    PLOT_WINDOW_SECONDS,
)

SENSOR_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
)


# ─────────────────────────────────────────────────────────────────────
# ZEROING
# The plotter is spawned fresh each time it is toggled on (D-pad up),
# so the first IMU sample after launch becomes the datum and every
# trace starts at 0. Toggle off/on to re-zero at the current attitude.
#
# Display only — teleop_controller subscribes to IMU_TOPIC directly and
# is unaffected by anything here.
#
# Yaw: absolute heading is arbitrary, so relative is almost always what
# you want. Leave True.
ZERO_YAW = True

# Roll/pitch: absolute IS meaningful — a few degrees of static nose-up
# from tether drag is a real sim-to-real discrepancy worth seeing.
# Zeroing hides it. Set False to keep roll/pitch absolute.
ZERO_ROLL_PITCH = True

class PlotterNode(Node):
    def __init__(self):
        super().__init__("imu_depth_plotter")

        self.lock = threading.Lock()
        self.start_time = None

        self.depth_t = deque()
        self.depth_v = deque()

        self.imu_t = deque()
        self.roll_v = deque()
        self.pitch_v = deque()
        self.yaw_v = deque()

        # (roll, pitch, yaw) in degrees, captured on the first IMU
        # sample. None until then.
        self.datum = None

        self.create_subscription(
            Float64,
            DEPTH_TOPIC,
            self.depth_callback,
            SENSOR_QOS,
        )

        self.create_subscription(
            Imu,
            IMU_TOPIC,
            self.imu_callback,
            SENSOR_QOS,
        )

        self.get_logger().info(
            f"Plotter running: {DEPTH_TOPIC} + {IMU_TOPIC}"
        )

    def _elapsed(self) -> float:
        now = self.get_clock().now().nanoseconds * 1e-9

        if self.start_time is None:
            self.start_time = now

        return now - self.start_time

    @staticmethod
    def _trim(times: deque, *series: deque):
        if not times:
            return

        cutoff = times[-1] - PLOT_WINDOW_SECONDS

        while times and times[0] < cutoff:
            times.popleft()
            for s in series:
                if s:
                    s.popleft()

    def depth_callback(self, msg: Float64):
        with self.lock:
            self.depth_t.append(self._elapsed())
            self.depth_v.append(float(msg.data))
            self._trim(self.depth_t, self.depth_v)

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        roll, pitch, yaw = quaternion_to_rpy(q.x, q.y, q.z, q.w)

        r = math.degrees(roll)
        p = math.degrees(pitch)
        y = math.degrees(yaw)

        with self.lock:
            if self.datum is None:
                self.datum = (r, p, y)

            if ZERO_ROLL_PITCH:
                # Roll, like yaw, crosses the atan2 boundary at +/-180.
                # Subtract the datum using the shortest angular difference.
                r = (r - self.datum[0] + 180.0) % 360.0 - 180.0
                p -= self.datum[1]

            if ZERO_YAW:
                # Wrap to +-180 so zeroing near the +-180 boundary does
                # not throw a full 360 jump into the trace.
                y = (y - self.datum[2] + 180.0) % 360.0 - 180.0

            self.imu_t.append(self._elapsed())
            self.roll_v.append(r)
            self.pitch_v.append(p)
            self.yaw_v.append(y)
            self._trim(self.imu_t, self.roll_v, self.pitch_v, self.yaw_v)

    def snapshot(self):
        with self.lock:
            return (
                list(self.depth_t),
                list(self.depth_v),
                list(self.imu_t),
                list(self.roll_v),
                list(self.pitch_v),
                list(self.yaw_v),
            )


def create_plot(snapshot):
    """Build the four panels and their update callback."""
    fig = plt.figure(figsize=(13, 7.5), layout="constrained")
    fig.canvas.manager.set_window_title("SubbyROV — Depth & IMU")
    grid = fig.add_gridspec(3, 2)
    ax_depth = fig.add_subplot(grid[:, 0])
    ax_pitch = fig.add_subplot(grid[0, 1], sharex=ax_depth)
    ax_roll = fig.add_subplot(grid[1, 1], sharex=ax_depth)
    ax_yaw = fig.add_subplot(grid[2, 1], sharex=ax_depth)

    depth_line, = ax_depth.plot([], [], color="tab:blue", label="depth")
    ax_depth.set_title("Depth")
    ax_depth.set_ylabel("Depth (m)")
    ax_depth.set_xlabel("Time (s)")
    ax_depth.invert_yaxis()
    ax_depth.grid(True, alpha=0.3)
    pitch_line, = ax_pitch.plot([], [], color="tab:green")
    roll_line, = ax_roll.plot([], [], color="tab:red")
    yaw_line, = ax_yaw.plot([], [], color="tab:orange")

    for axis, title, relative in (
        (ax_pitch, "Pitch", ZERO_ROLL_PITCH),
        (ax_roll, "Roll", ZERO_ROLL_PITCH),
        (ax_yaw, "Yaw", ZERO_YAW),
    ):
        axis.set_title(title)
        axis.set_ylabel("Relative (°)" if relative else "Angle (°)")
        axis.grid(True, alpha=0.3)
        axis.set_ylim(-5.0, 5.0)
    ax_pitch.tick_params(labelbottom=False)
    ax_roll.tick_params(labelbottom=False)
    ax_yaw.set_xlabel("Time (s)")

    def update(_frame):
        dt_, dv, it, rv, pv, yv = snapshot()

        depth_line.set_data(dt_, dv)
        roll_line.set_data(it, rv)
        pitch_line.set_data(it, pv)
        yaw_line.set_data(it, yv)

        latest = max(
            dt_[-1] if dt_ else 0.0,
            it[-1] if it else 0.0,
        )
        left = max(0.0, latest - PLOT_WINDOW_SECONDS)
        right = max(left + 1.0, latest)

        # All four panels share time; each measurement has its own scale.
        ax_depth.set_xlim(left, right)

        if dv:
            lo, hi = min(dv), max(dv)
            pad = max(0.1, (hi - lo) * 0.2)
            ax_depth.set_ylim(hi + pad, lo - pad)   # inverted

        for axis, values in ((ax_pitch, pv), (ax_roll, rv), (ax_yaw, yv)):
            if values:
                lo, hi = min(values), max(values)
                pad = max(5.0, (hi - lo) * 0.15)
                axis.set_ylim(lo - pad, hi + pad)

        return depth_line, roll_line, pitch_line, yaw_line

    return fig, update


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = PlotterNode()

    def spin():
        try:
            rclpy.spin(node)
        except rclpy.executors.ExternalShutdownException:
            pass

    executor_thread = threading.Thread(target=spin, daemon=True)
    executor_thread.start()
    fig, update = create_plot(node.snapshot)

    animation = FuncAnimation(  # noqa: F841 - keep animation alive until plt.show returns
        fig,
        update,
        interval=PLOT_REFRESH_MS,
        cache_frame_data=False,
    )

    def shutdown(signum, frame):
        plt.close("all")


    # ViewerManager sends SIGINT on the next D-pad up press.
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGHUP, shutdown)

    try:
        plt.show()

    except KeyboardInterrupt:
        pass

    finally:
        if rclpy.ok():
            rclpy.shutdown()
        executor_thread.join(timeout=2.0)
        node.destroy_node()


if __name__ == "__main__":
    main()
