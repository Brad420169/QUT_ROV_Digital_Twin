"""Input freshness measured with monotonic time, independent of ROS clock resets."""
import time


class Freshness:
    def __init__(self, timeout, clock=time.monotonic):
        self.timeout = timeout
        self.clock = clock
        self.last_seen = None

    def touch(self):
        self.last_seen = self.clock()

    def invalidate(self):
        self.last_seen = None

    def fresh(self):
        return self.last_seen is not None and 0 <= self.clock() - self.last_seen <= self.timeout
