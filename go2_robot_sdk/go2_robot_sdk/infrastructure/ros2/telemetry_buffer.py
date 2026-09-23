"""Bounded handoff from WebRTC reception to the ROS executor."""
from threading import Lock


class LatestTelemetry:
    """Retain one sample per robot/topic; never replay a backlog of sensor poses."""

    def __init__(self, rates):
        if any(rate <= 0 for rate in rates.values()):
            raise ValueError('Telemetry rates must be positive')
        self.intervals = {topic: 1.0 / rate for topic, rate in rates.items()}
        self.pending = {}
        self.next_due = {}
        self.lock = Lock()

    def put(self, robot_id, topic, message):
        if topic not in self.intervals:
            return False
        with self.lock:
            self.pending[(robot_id, topic)] = message
        return True

    def take_ready(self, now):
        ready = []
        with self.lock:
            for key in list(self.pending):
                if now >= self.next_due.get(key, 0.0):
                    ready.append((key[0], self.pending.pop(key)))
                    self.next_due[key] = now + self.intervals[key[1]]
        return ready
