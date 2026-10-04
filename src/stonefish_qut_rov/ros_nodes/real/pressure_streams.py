#!/usr/bin/env python3
"""Expose raw MAVLink pressure streams as diagnostic ROS FluidPressure topics."""
import math
import struct
import sys

STREAMS = {29: 'scaled_pressure', 137: 'scaled_pressure2', 143: 'scaled_pressure3'}


def pressure_pa(payload64, length):
    payload = b''.join(int(word).to_bytes(8, 'little') for word in payload64)
    if not 1 <= length <= len(payload):
        raise ValueError('Invalid MAVLink payload length')
    # MAVLink 2 trims trailing zero bytes, including bytes within float fields.
    value = struct.unpack_from('<f', payload[:length].ljust(8, b'\0'), 4)[0] * 100.0
    if not math.isfinite(value) or value <= 0:
        raise ValueError('Invalid absolute pressure')
    return value


def main():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from mavros_msgs.msg import Mavlink
    from mavros_msgs.srv import MessageInterval
    from sensor_msgs.msg import FluidPressure

    rclpy.init()
    node = rclpy.create_node('pressure_streams')
    publishers = {mid: node.create_publisher(FluidPressure, '/qut_rov/diagnostics/' + name, 10)
                  for mid, name in STREAMS.items()}
    subscriptions = {}

    def receive(msg):
        if msg.msgid not in STREAMS or msg.framing_status != Mavlink.FRAMING_OK:
            return
        try:
            value = pressure_pa(msg.payload64, msg.len)
        except ValueError:
            return
        # Keep different vehicle/component sources separate; never mix sensors.
        key = (msg.sysid, msg.compid, msg.msgid)
        if key not in subscriptions:
            node.get_logger().info(f'Receiving {STREAMS[msg.msgid]} from system {msg.sysid}, component {msg.compid}')
            subscriptions[key] = True
        if msg.sysid != 1 or msg.compid != 1:
            return
        out = FluidPressure()
        out.header = msg.header
        out.fluid_pressure = value
        publishers[msg.msgid].publish(out)

    def discover():
        for topic, types in node.get_topic_names_and_types():
            if 'mavros_msgs/msg/Mavlink' in types and topic.endswith('/mavlink_source') and topic not in subscriptions:
                subscriptions[topic] = node.create_subscription(Mavlink, topic, receive, qos_profile_sensor_data)
                node.get_logger().info(f'Listening on {topic}; publishing system 1/component 1 pressure in Pa')

    node.create_timer(1.0, discover)
    client = node.create_client(MessageInterval, '/mavros/set_message_interval')
    try:
        if client.wait_for_service(timeout_sec=5):
            for mid in STREAMS:
                future = client.call_async(MessageInterval.Request(message_id=mid, message_rate=30.0 if mid == 29 else 5.0))
                rclpy.spin_until_future_complete(node, future, timeout_sec=5)
                if not future.done():
                    node.get_logger().warning('Rate request timed out; stopping requests to avoid overlap.')
                    break
                if not future.result() or not future.result().success:
                    node.get_logger().warning(f'Rate request rejected for {STREAMS[mid]}')
        else:
            node.get_logger().warning('Rate service unavailable; listening to existing streams.')
        node.get_logger().info('Lower the disarmed ROV: +0.5 m should produce approximately +4895 Pa. Ctrl-C to stop.')
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    if '--self-test' in sys.argv:
        data = struct.pack('<Iffh', 1000, 1013.25, 0.0, 2000)
        words = [int.from_bytes(data[i:i+8].ljust(8, b'\0'), 'little') for i in range(0, len(data), 8)]
        assert pressure_pa(words, len(data)) == 101325.0
        assert pressure_pa([int.from_bytes(struct.pack('<If', 0, 1024.0), 'little')], 8) == 102400.0
        try:
            pressure_pa([], 0)
        except ValueError:
            pass
        else:
            raise AssertionError('Empty payload accepted')
        print('Pressure decoder checks passed')
    else:
        main()
