"""Offline protocol checks; these tests never connect to the vehicle."""
import binascii
from pathlib import Path
import struct
import sys
import unittest
import time
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros_nodes/modular_architecture'))
from gimbal_fpv import Connection, make_packet, FPVHold
from gimbal_motor import MotorControl, CONTROLLER_SHA256


class Socket:
    def __init__(self, chunks):
        self.chunks = iter(chunks)
    def sendall(self, data):
        self.sent = data
    def settimeout(self, seconds):
        pass
    def recv(self, size):
        return next(self.chunks, b'')


class ProtocolTests(unittest.TestCase):
    def test_window_motor_lifecycle(self):
        calls = []

        class Motors:
            def __init__(self, host):
                pass
            def set_enabled(self, enabled):
                calls.append(enabled)

        class Link:
            def __init__(self, sock):
                pass
            def exchange(self, packet):
                reply = bytearray(72)
                reply[5] = 0x1c
                return reply

        def wait_for(predicate):
            deadline = time.monotonic() + 2
            while not predicate() and time.monotonic() < deadline:
                time.sleep(.01)
            self.assertTrue(predicate(), calls)

        with patch('gimbal_fpv.MotorControl', Motors), \
             patch('gimbal_fpv.socket.create_connection'), \
             patch('gimbal_fpv.Connection', Link):
            worker = FPVHold('192.168.144.108', visible=False, log=lambda _: None)
            try:
                wait_for(lambda: calls == [False])
                worker.set_visible(True)
                wait_for(lambda: calls == [False, True])
                worker.set_visible(False)
                wait_for(lambda: calls == [False, True, False])
                worker.set_visible(True)
                wait_for(lambda: calls == [False, True, False, True])
            finally:
                worker.stop()
            self.assertFalse(worker.thread.is_alive())
            self.assertEqual(calls, [False, True, False, True, False])

    def test_remote_command_checks_firmware_and_rediscovers_pid(self):
        motor = MotorControl('192.168.144.108')
        motor.remote = '/tmp/test-helper'
        with patch.object(motor, 'run', return_value=b'') as run:
            motor.set_enabled(False)
            command = run.call_args.args[0][-1]
            self.assertIn(CONTROLLER_SHA256, command)
            self.assertIn('pidof gb_control', command)
            self.assertTrue(command.endswith('"$1" stop'))
        with patch.object(motor, 'run', side_effect=OSError('disconnected')):
            with self.assertRaises(OSError):
                motor.set_enabled(True)
        self.assertIsNone(motor.remote)

    def test_target_encoding(self):
        p = make_packet(90, -90, 0x1c)
        self.assertEqual(len(p), 72)
        self.assertEqual(struct.unpack_from('<hhh', p, 5), (0, 9000, -9000))
        self.assertEqual(p[11], 4)
        self.assertEqual(p[69], 0x1c)
        self.assertEqual(int.from_bytes(p[-2:], 'big'), binascii.crc_hqx(p[:-2], 0))
        p = make_packet(-30, 90, roll=12)
        self.assertEqual(struct.unpack_from('<hhh', p, 5), (1200, -3000, 9000))

    def test_nudge_is_hidden_gated_and_clamped(self):
        import threading
        worker = FPVHold.__new__(FPVHold)
        worker.visible = threading.Event()
        worker.roll, worker.pitch, worker.yaw = 0., -90., 90.
        worker.log = lambda _: None
        worker.nudge(2, 2)
        self.assertEqual((worker.roll, worker.pitch), (0., -90.))
        worker.visible.set()
        worker.nudge(2, 2)
        self.assertEqual(struct.unpack_from('<hhh', worker.target, 5), (200, -8800, 9000))
        worker.nudge(100, 300)
        self.assertEqual((worker.roll, worker.pitch), (45., 90.))
        worker.pitch = -90.
        worker.nudge(0, -2)
        self.assertEqual(struct.unpack_from('<h', worker.target, 7)[0], -9200)
        worker.nudge(0, -100)
        self.assertEqual(worker.pitch, -145.)
        self.assertEqual(struct.unpack_from('<h', worker.target, 7)[0], -14500)

    def test_fragmented_and_combined_replies(self):
        p = bytearray(make_packet(0, 0))
        p[:2] = b'\x8a\x5e'
        p[5] = 0x1c
        p[-2:] = binascii.crc_hqx(p[:-2], 0).to_bytes(2, 'big')
        p = bytes(p)
        link = Connection(Socket([p[:1], p[1:20], p[20:] + p]))
        self.assertEqual(link.exchange(b''), p)
        self.assertEqual(link.exchange(b''), p)

    def test_reject_bad_crc(self):
        p = bytearray(make_packet(0, 0))
        p[:2] = b'\x8a\x5e'
        with self.assertRaises(ValueError):
            Connection(Socket([bytes(p)])).exchange(b'')

    def test_bounds(self):
        for pitch, yaw in [(float('nan'), 0), (0, 91), (-146, 0), (91, 0)]:
            with self.assertRaises(ValueError):
                make_packet(pitch, yaw)

if __name__ == '__main__':
    unittest.main()
