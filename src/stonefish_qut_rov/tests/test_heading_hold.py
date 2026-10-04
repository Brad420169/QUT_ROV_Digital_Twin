import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
from unittest.mock import Mock

root = Path(__file__).resolve().parents[1] / 'ros_nodes'
sys.path[:0] = [str(root / 'common'), str(root / 'real')]
from pid_controller import PIDController
from teleop_controller import GamepadTeleop
from sensor_msgs.msg import Imu


def test_heading_hold():
    pid = PIDController(.2, .1, .1, wrap_angle=True, deadband=math.radians(5))
    for angle in (-4, 0, 4):
        assert pid.compute(0, math.radians(angle), .1) == 0
    assert pid.compute(math.radians(179), math.radians(-179), .1) == 0
    assert pid.compute(0, math.radians(20), .1) < 0
    assert pid.compute(0, 0, .1) == 0
    assert pid._integral == 0 and pid._prev_measurement is None
    assert pid.compute(0, math.radians(-20), .1) > 0
    assert PIDController(1, 0, 0).compute(.1, 0, .1) == .1

    node = NS(rov_mode="real", fish_follow_mode=False, trajectory_mode=False, depth_keeping=False,
              current_depth=.4, current_yaw=.2, inputs={k: Mock() for k in ('depth', 'imu')},
              depth_pid=Mock(), yaw_pid=pid, get_logger=Mock())
    for item in node.inputs.values(): item.fresh.return_value = True
    GamepadTeleop.toggle_depth_keeping(node)
    assert node.depth_keeping and node.target_depth == .4 and node.target_yaw == .2
    node.prev_yaw_time = NS(nanoseconds=0)
    class Stamp:
        nanoseconds = 100000000
        def __sub__(self, other): return NS(nanoseconds=self.nanoseconds-other.nanoseconds)
    node.get_clock = lambda: NS(now=Stamp)
    msg = Imu()
    msg.orientation.w = math.cos(.5 / 2)
    msg.orientation.z = math.sin(.5 / 2)
    GamepadTeleop.imu_callback(node, msg)
    assert node.yaw_cmd < 0  # Heading hold runs even without trajectory mode.
    GamepadTeleop.toggle_depth_keeping(node)
    assert not node.depth_keeping and node.target_yaw is None and node.yaw_cmd == 0
    node.inputs['imu'].fresh.return_value = False
    GamepadTeleop.toggle_depth_keeping(node)
    assert not node.depth_keeping


if __name__ == '__main__':
    test_heading_hold()
    print('Heading deadband, wraparound, hold enable/disable and IMU checks passed')
