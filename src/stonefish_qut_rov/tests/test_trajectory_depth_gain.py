"""Run after sourcing ROS: verifies D selection on the real depth callback."""
from pathlib import Path
import sys
from types import SimpleNamespace as NS
from unittest.mock import Mock
root = Path(__file__).resolve().parents[1] / 'ros_nodes'
sys.path[:0] = [str(root / 'common'), str(root / 'real')]
from teleop_controller import GamepadTeleop
from pid_controller import PIDController


def test_trajectory_depth_gain():
    class Stamp:
        nanoseconds = 100000000
        def __sub__(self, other): return NS(nanoseconds=self.nanoseconds-other.nanoseconds)
    node = NS(inputs={'depth': Mock()}, fish_follow_mode=False, target_depth=1.,
              depth_pid=PIDController(0, 0, 0), k_depth_kd=.1, k_depth_traj_kd=.3,
              k_depth_ff=0., get_clock=lambda: NS(now=Stamp))
    node.inputs['depth'].fresh.return_value = True
    # Station -> trajectory -> station must restore the original damping.
    for trajectory, expected in [(False, .1), (True, .3), (False, .1)]:
        node.depth_keeping = not trajectory
        node.trajectory_mode = trajectory
        node.prev_depth_time = NS(nanoseconds=0)
        node.depth_pid.reset()
        node.depth_pid.compute(1., 1., .1)
        GamepadTeleop.depth_callback(node, NS(data=1.1))
        assert node.depth_pid.kd == expected
        assert abs(node.heave_cmd - expected) < 1e-9


if __name__ == '__main__':
    test_trajectory_depth_gain()
    print('Trajectory D selection and station-keeping restoration passed')
