"""Exercise held camera input without ROS initialization or vehicle connections."""
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros_nodes/modular_architecture'))
from camera_viewer_rtsp import ViewerNode


def test_hold_release_hidden_and_stale():
    node = SimpleNamespace(window_open=True, show=True, gimbal=Mock(),
                           _dpad_last=None, _dpad_stamp=float('-inf'), _gimbal_tick=0.)
    def joy(x, y):
        ViewerNode.gimbal_joy_callback(node, SimpleNamespace(axes=[0.] * 6 + [x, y]))
    with patch('camera_viewer_rtsp.time.monotonic') as clock:
        clock.return_value = 0.
        joy(0, 1)  # Must release after opening.
        ViewerNode.update_gimbal_aim(node)
        node.gimbal.nudge.assert_not_called()
        joy(0, 0)
        joy(1, 1)
        for tick in (.025, .05):
            clock.return_value = tick
            ViewerNode.update_gimbal_aim(node)
        assert node.gimbal.nudge.call_count == 2
        node.gimbal.nudge.assert_called_with(-.5, .5)
        joy(0, 0)
        clock.return_value = .075
        ViewerNode.update_gimbal_aim(node)
        assert node.gimbal.nudge.call_count == 2
        joy(0, -1)
        node.show = False
        clock.return_value = .1
        ViewerNode.update_gimbal_aim(node)
        assert node.gimbal.nudge.call_count == 2
        node.show = True
        clock.return_value = 1.
        ViewerNode.update_gimbal_aim(node)
        joy(0, -1)
        ViewerNode.update_gimbal_aim(node)
        assert node.gimbal.nudge.call_count == 2
        joy(0, 0)
        joy(0, -1)
        clock.return_value = 1.025
        ViewerNode.update_gimbal_aim(node)
        assert node.gimbal.nudge.call_count == 3
