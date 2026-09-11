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

# ─────────────────────────────────────────────────────────────────────
# WHICH TRACES TO PLOT
# Data for all three is always collected — these only control what is
# drawn, so flipping one back on needs no other change.
SHOW_ROLL  = False
SHOW_PITCH = True
SHOW_YAW   = False


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
        roll, pitch = quaternion_to_roll_pitch(q.x, q.y, q.z, q.w)
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

        r = math.degrees(roll)
        p = math.degrees(pitch)
        y = math.degrees(yaw)

        with self.lock:
            if self.datum is None:
                self.datum = (r, p, y)

            if ZERO_ROLL_PITCH:
                r -= self.datum[0]
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

    roll_line,  = ax_imu.plot([], [], color="tab:red",    label="roll")
    pitch_line, = ax_imu.plot([], [], color="tab:green",  label="pitch")
    yaw_line,   = ax_imu.plot([], [], color="tab:orange", label="yaw")

    roll_line.set_visible(SHOW_ROLL)
    pitch_line.set_visible(SHOW_PITCH)
    yaw_line.set_visible(SHOW_YAW)

    # Label only reflects the traces actually shown
    _rp = "rel" if ZERO_ROLL_PITCH else "abs"
    _y = "rel" if ZERO_YAW else "abs"
    _bits = []
    if SHOW_ROLL:
        _bits.append(f"roll {_rp}")
    if SHOW_PITCH:
        _bits.append(f"pitch {_rp}")
    if SHOW_YAW:
        _bits.append(f"yaw {_y}")
    ax_imu.set_ylabel(
        "Angle (deg)  " + ", ".join(_bits) if _bits else "Angle (deg)"
    )

    ax_imu.set_xlabel("Time (s)")
    ax_imu.grid(True, alpha=0.3)

    # Legend carries only the visible traces
    _handles = [
        ln for ln, on in (
            (roll_line, SHOW_ROLL),
            (pitch_line, SHOW_PITCH),
            (yaw_line, SHOW_YAW),
        ) if on
    ]
    if _handles:
        ax_imu.legend(handles=_handles, loc="upper right")

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

        # Autoscale on visible traces only — a hidden yaw swing must not
        # squash the pitch trace.
        allv = []
        if SHOW_ROLL:
            allv += rv
        if SHOW_PITCH:
            allv += pv
        if SHOW_YAW:
            allv += yv

        if allv:
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