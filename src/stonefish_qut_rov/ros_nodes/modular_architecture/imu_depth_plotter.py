#!/usr/bin/env python3
"""
Live IMU + depth plotter.

Subscribes to the COMMON topics, so it works unchanged in both sim and
real mode:
    /qut_rov/depth   (std_msgs/Float64, metres, positive down)
    /qut_rov/imu     (sensor_msgs/Imu)

Displays a rolling window:
    top    — depth vs time (y axis inverted, deeper is lower)
    bottom — roll / pitch / yaw vs time, degrees

Launched and killed by teleop_controller via the D-pad up toggle, but
can also be run standalone:
    ros2 run stonefish_qut_rov imu_depth_plotter.py
"""

import math
import signal
import sys
import threading
from collections import deque

import matplotlib
import matplotlib.pyplot as plt
import rclpy
from matplotlib.animation import FuncAnimation
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

from control_utils import quaternion_to_yaw
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


def quaternion_to_roll_pitch(x: float, y: float, z: float, w: float):
    """Roll and pitch in radians (yaw comes from control_utils)."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    return roll, pitch


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
        roll, pitch = quaternion_to_roll_pitch(q.x, q.y, q.z, q.w)
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

        with self.lock:
            self.imu_t.append(self._elapsed())
            self.roll_v.append(math.degrees(roll))
            self.pitch_v.append(math.degrees(pitch))
            self.yaw_v.append(math.degrees(yaw))
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


def main(args=None):
    rclpy.init(args=args)
    node = PlotterNode()

    executor_thread = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True,
    )
    executor_thread.start()

    fig, (ax_depth, ax_imu) = plt.subplots(1, 2, figsize=(13, 6.5))
    fig.canvas.manager.set_window_title("SubbyROV — Depth & IMU")

    # Force both panels square regardless of window resizing
    ax_depth.set_box_aspect(1)
    ax_imu.set_box_aspect(1)

    depth_line, = ax_depth.plot([], [], color="tab:blue", label="depth")
    ax_depth.set_ylabel("Depth (m)")
    ax_depth.set_xlabel("Time (s)")
    ax_depth.invert_yaxis()
    ax_depth.grid(True, alpha=0.3)
    ax_depth.legend(loc="upper right")

    roll_line,  = ax_imu.plot([], [], color="tab:red",   label="roll")
    pitch_line, = ax_imu.plot([], [], color="tab:green", label="pitch")
    yaw_line,   = ax_imu.plot([], [], color="tab:orange", label="yaw")
    ax_imu.set_ylabel("Angle (deg)")
    ax_imu.set_xlabel("Time (s)")
    ax_imu.grid(True, alpha=0.3)
    ax_imu.legend(loc="upper right")

    def update(_frame):
        dt_, dv, it, rv, pv, yv = node.snapshot()

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

        # Shared time window, but each panel carries its own axis
        ax_depth.set_xlim(left, right)
        ax_imu.set_xlim(left, right)

        if dv:
            lo, hi = min(dv), max(dv)
            pad = max(0.1, (hi - lo) * 0.2)
            ax_depth.set_ylim(hi + pad, lo - pad)   # inverted

        if rv or pv or yv:
            allv = rv + pv + yv
            lo, hi = min(allv), max(allv)
            pad = max(5.0, (hi - lo) * 0.15)
            ax_imu.set_ylim(lo - pad, hi + pad)

        return depth_line, roll_line, pitch_line, yaw_line

    fig.tight_layout()

    animation = FuncAnimation(
        fig,
        update,
        interval=PLOT_REFRESH_MS,
        cache_frame_data=False,
    )

    def shutdown(signum, frame):
        plt.close("all")
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGHUP, shutdown)

    try:
        plt.show()

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
