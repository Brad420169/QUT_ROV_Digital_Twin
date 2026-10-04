"""Run directly after sourcing ROS; no vehicle or ROS node is started."""
import math
from pathlib import Path
import struct
import sys
from types import SimpleNamespace
from unittest.mock import patch

root = Path(__file__).resolve().parents[1] / 'ros_nodes'
sys.path[:0] = [str(root / 'common'), str(root / 'real')]
from mavros_msgs.msg import Mavlink
from real_interface import RealInterface
from rov_config import REAL_FAST_STREAM_MESSAGE_IDS


def test_stream3():
    published = []
    pressure = []
    node = SimpleNamespace(surface_pressure_pa=None, _pressure_samples=[],
                           _pressure_sample_time=None,
                           depth_pub=SimpleNamespace(publish=lambda m: published.append(m.data)),
                           pressure_pub=SimpleNamespace(publish=lambda m: pressure.append(m.fluid_pressure)),
                           get_logger=lambda: SimpleNamespace(info=lambda *a: None, warning=lambda *a: None))
    node.pressure_callback = lambda m: RealInterface.pressure_callback(node, m)

    def packet(pa, mid=143, sysid=1, framing=1):
        raw = struct.pack('<Iffh', 1000, pa / 100, 0, 2000).rstrip(b'\0')
        return Mavlink(msgid=mid, sysid=sysid, compid=1, framing_status=framing,
                       len=len(raw), payload64=[int.from_bytes(raw[i:i+8].ljust(8, b'\0'), 'little')
                                              for i in range(0, len(raw), 8)])

    for msg in [packet(98020, mid=29), packet(98020, mid=137),
                packet(98020, sysid=2), packet(98020, framing=2)]:
        RealInterface.mavlink_pressure_callback(node, msg)
    assert not pressure and not published
    with patch('real_interface.time.monotonic', side_effect=[i * .12 for i in range(30)]):
        for _ in range(30):
            RealInterface.mavlink_pressure_callback(node, packet(98020))
    assert abs(published[-1]) < 1e-9
    RealInterface.mavlink_pressure_callback(node, packet(100910))
    assert math.isclose(published[-1], 2890 / (998 * 9.81), abs_tol=1e-5)
    count = len(published)
    for value in [float('nan'), float('inf'), -100, 0]:
        RealInterface.mavlink_pressure_callback(node, packet(value))
    RealInterface.mavlink_pressure_callback(node, Mavlink(msgid=143, sysid=1, compid=1, framing_status=1, len=14))
    assert len(published) == count
    assert 143 in REAL_FAST_STREAM_MESSAGE_IDS and 29 not in REAL_FAST_STREAM_MESSAGE_IDS


def test_construction():
    # Exercise __init__ without connecting to ROS or commanding the vehicle.
    from rclpy.node import Node
    from rov_config import REAL_MAVLINK_TOPIC, REAL_PRESSURE_TOPIC
    with patch.object(Node, '__init__', return_value=None), \
         patch.object(Node, 'get_logger'), \
         patch.object(Node, 'create_publisher', autospec=True) as publishers, \
         patch.object(Node, 'create_subscription', autospec=True) as subscriptions, \
         patch.object(Node, 'create_timer', autospec=True):
        node = RealInterface()
        calls = [call.args for call in subscriptions.call_args_list]
        assert any(args[1] is Mavlink and args[2] == REAL_MAVLINK_TOPIC
                   and args[3] == node.mavlink_pressure_callback for args in calls)
        assert any(call.args[2] == REAL_PRESSURE_TOPIC for call in publishers.call_args_list)
        assert node.pressure_pub is not None


if __name__ == '__main__':
    test_stream3()
    test_construction()
    print('Stream 3 filtering, calibration, depth conversion and invalid input checks passed')
