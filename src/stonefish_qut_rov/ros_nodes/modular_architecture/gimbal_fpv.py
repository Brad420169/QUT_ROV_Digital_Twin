"""XF Z-1 Mini body-relative FPV hold over TCP; no ROS dependencies."""
import binascii
import math
import socket
import struct
import threading
import time


def make_packet(pitch, yaw, order=0):
    if not all(math.isfinite(v) and -90 <= v <= 90 for v in (pitch, yaw)):
        raise ValueError('FPV targets must be finite and within ±90 degrees')
    data = bytearray(70)
    data[:5] = bytes.fromhex('a8 e5 48 00 02')
    struct.pack_into('<hhh', data, 5, 0, round(pitch * 100), round(yaw * 100))
    data[11] = 4  # Valid angles, no fabricated carrier INS data.
    data[30] = 1
    data[69] = order
    return bytes(data) + binascii.crc_hqx(data, 0).to_bytes(2, 'big')


class Connection:
    def __init__(self, sock):
        self.sock = sock
        self.buffer = b''

    def exchange(self, packet):
        self.sock.sendall(packet)
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            start = self.buffer.find(b'\x8a\x5e')
            if start >= 0:
                self.buffer = self.buffer[start:]
                if len(self.buffer) >= 4:
                    size = int.from_bytes(self.buffer[2:4], 'little')
                    if not 72 <= size <= 4096:
                        raise ValueError('Invalid GCU response length')
                    if len(self.buffer) >= size:
                        data, self.buffer = self.buffer[:size], self.buffer[size:]
                        if binascii.crc_hqx(data[:-2], 0) != int.from_bytes(data[-2:], 'big'):
                            raise ValueError('Invalid GCU response checksum')
                        return data
            self.sock.settimeout(max(.001, deadline - time.monotonic()))
            data = self.sock.recv(4096)
            if not data:
                raise ConnectionError('Gimbal disconnected')
            self.buffer += data
            if len(self.buffer) > 8192:
                raise ValueError('Unframed GCU response')
        raise TimeoutError('Gimbal response timed out')


class FPVHold:
    """Maintain the target at 40 Hz; retry dropped connections every two seconds.

    Closing leaves the last target in the camera, rather than sending zero
    angles (which would command a different pose). This does not save flash.
    """
    def __init__(self, host, pitch=90., yaw=-90., log=print):
        self.host, self.log = host, log
        self.target = make_packet(pitch, yaw)
        self.mode = make_packet(pitch, yaw, 0x1c)
        # Mode-independent query: never interpret target angles as rate commands.
        query = bytearray(self.target[:-2])
        query[5:12] = bytes(7)
        self.query = bytes(query) + binascii.crc_hqx(query, 0).to_bytes(2, 'big')
        self.done = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        failure_logged = False
        while not self.done.is_set():
            try:
                with socket.create_connection((self.host, 2332), timeout=1) as sock:
                    link = Connection(sock)
                    link.exchange(self.query)  # Separate repeated mode commands.
                    ack = link.exchange(self.mode)
                    if ack[69] == 0x1c and len(ack) > 72 and ack[70] != 0:
                        raise ValueError('Camera rejected FPV mode')
                    ready = False
                    for _ in range(20):
                        if self.done.wait(.025):
                            return
                        status = link.exchange(self.target)
                        if status[5] == 0x1c:
                            ready = True
                            break
                    if not ready:
                        raise ValueError('FPV mode not confirmed')
                    self.log('Gimbal FPV hold active (body-relative target).')
                    failure_logged = False
                    while not self.done.is_set():
                        tick = time.monotonic()
                        status = link.exchange(self.target)
                        if status[5] != 0x1c:
                            raise ValueError('Gimbal mode overridden; check other controllers')
                        self.done.wait(max(0, .025 - (time.monotonic() - tick)))
            except (OSError, ValueError) as exc:
                if not self.done.is_set() and not failure_logged:
                    self.log(f'Gimbal hold unavailable: {exc}; retrying in 2 seconds.')
                    failure_logged = True
                self.done.wait(2)

    def stop(self):
        self.done.set()
        self.thread.join(timeout=3)
