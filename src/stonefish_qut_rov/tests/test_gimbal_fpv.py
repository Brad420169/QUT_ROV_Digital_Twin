"""Offline protocol checks; these tests never connect to the vehicle."""
import binascii
from pathlib import Path
import struct
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros_nodes/modular_architecture'))
from gimbal_fpv import Connection, make_packet


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
    def test_target_encoding(self):
        p = make_packet(90, -90, 0x1c)
        self.assertEqual(len(p), 72)
        self.assertEqual(struct.unpack_from('<hhh', p, 5), (0, 9000, -9000))
        self.assertEqual(p[11], 4)
        self.assertEqual(p[69], 0x1c)
        self.assertEqual(int.from_bytes(p[-2:], 'big'), binascii.crc_hqx(p[:-2], 0))

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
        for pitch, yaw in [(float('nan'), 0), (0, 91), (-91, 0)]:
            with self.assertRaises(ValueError):
                make_packet(pitch, yaw)

if __name__ == '__main__':
    unittest.main()
