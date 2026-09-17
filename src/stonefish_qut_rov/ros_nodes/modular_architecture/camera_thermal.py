"""Fail-closed camera health and read-only SSH temperature sampling."""
import math
import os
import threading
import time


class VisionGate:
    def __init__(self):
        self.hot = None

    def evaluate(self, temperature, age, fps, frame_age):
        if temperature is None or not math.isfinite(temperature) or age > 8:
            return False, 'Camera temperature unavailable; all vision control disabled.'
        # On first contact, a camera between 75 and 80 may already be cooling
        # from a thermal trip. Do not clear it until below the recovery threshold.
        if self.hot is None:
            self.hot = temperature >= 79.5
        if temperature >= 80:
            self.hot = True
        elif temperature < 79.5:
            self.hot = False
        if self.hot:
            return False, (f'Camera is at max temp (80 C) / cooling: {temperature:.1f} C. '
                           f'Thermal mode: 5 fps; measured {fps:.1f} fps. '
                           'All vision control disabled until below 79.5 C.')
        if frame_age > 1 or fps < 10:
            return False, f'Camera feed slow or unavailable ({fps:.1f} fps); all vision control disabled.'
        return True, f'Camera ready: {temperature:.1f} C, {fps:.1f} fps. Vision permitted, not automatically enabled.'


class VisionPermission:
    """Consumers must check this before enabling vision AND before outputting commands.

    Use a live (volatile) ROS subscription: never authorize from a stale latched
    True. No heartbeat for 2 seconds revokes permission.
    """
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.allowed = False
        self.stamp = float('-inf')

    def update(self, allowed):
        self.allowed, self.stamp = bool(allowed), self.clock()

    def valid(self):
        return self.allowed and self.clock()-self.stamp < 2


def read_temperature(host):
    """Use existing SSH keys, or CAMERA_SSH_PASSWORD; never save credentials."""
    import pexpect
    args = ['-o', 'StrictHostKeyChecking=yes', '-o', 'UpdateHostKeys=no',
            '-o', 'ConnectTimeout=2', '-o', 'NumberOfPasswordPrompts=1',
            'root@' + host, 'cat /sys/class/thermal/thermal_zone0/temp']
    child = pexpect.spawn('ssh', args, encoding='utf-8', timeout=3)
    try:
        event = child.expect(['[Pp]assword:', pexpect.EOF])
        if event == 0:
            password = os.environ.get('CAMERA_SSH_PASSWORD')
            if not password:
                raise RuntimeError('Set CAMERA_SSH_PASSWORD or configure SSH key access')
            child.sendline(password)
            child.expect(pexpect.EOF)
        result = child.before
        child.close()
        if child.exitstatus != 0:
            raise RuntimeError('Camera SSH temperature read failed')
        lines = [line.strip() for line in result.splitlines() if line.strip().lstrip('-').isdigit()]
        if len(lines) != 1:
            raise ValueError('Invalid temperature reply')
        value = int(lines[0]) / 1000
        if not -20 <= value <= 150:
            raise ValueError('Temperature outside expected range')
        return value
    finally:
        child.close(force=True)


class TemperatureMonitor:
    def __init__(self, host):
        self.host = host
        self.sample = (None, 0., 'Waiting for temperature')
        self.done = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        while not self.done.is_set():
            try:
                value = read_temperature(self.host)
                self.sample = (value, time.monotonic(), '')
            except Exception as exc:
                # Invalidate immediately, rather than retaining a misleading
                # last-good sample during a failed connection.
                self.sample = (None, 0., str(exc))
            self.done.wait(2)

    def stop(self):
        self.done.set()
        self.thread.join(timeout=7)
