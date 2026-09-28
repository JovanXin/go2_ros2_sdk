"""Bounded handoff from WebRTC reception to the ROS executor."""
from threading import Lock
import time


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

    def earliest_due(self):
        """Next pending deadline, or None when idle (not a stale sample replay)."""
        with self.lock:
            return min((self.next_due.get(key, 0.0) for key in self.pending),
                       default=None)

    def take_ready(self, now):
        ready = []
        with self.lock:
            for key in list(self.pending):
                if now >= self.next_due.get(key, 0.0):
                    ready.append((key[0], self.pending.pop(key)))
                    self.next_due[key] = now + self.intervals[key[1]]
        return ready


class TelemetryWakeup:
    """Asyncio-owned one-shot deadline -> ROS guard, with one wake in flight.

    Call received/published/close on the WebRTC loop only. The ROS executor
    acknowledges a completed drain using loop.call_soon_threadsafe(published).
    Arrivals replace pending samples without waking ROS at the source rate.
    An early final arrival gets a timer even if no subsequent packet arrives.
    """

    def __init__(self, buffer, loop, wake, clock=time.monotonic):
        self.buffer = buffer
        self.loop = loop
        self.wake = wake
        self.clock = clock
        self.handle = None
        self.deadline = None
        self.in_flight = False
        self.closed = False

    def received(self):
        if self.closed or self.in_flight:
            return
        deadline = self.buffer.earliest_due()
        if deadline is None:
            return
        if self.handle is not None:
            if self.deadline <= deadline:
                return
            self.handle.cancel()
        self.deadline = deadline
        self.handle = self.loop.call_later(
            max(0.0, deadline - self.clock()), self._fire)

    def _fire(self):
        self.handle = None
        self.deadline = None
        if not self.closed:
            self.in_flight = True
            self.wake()

    def published(self):
        self.in_flight = False
        self.received()

    def close(self):
        self.closed = True
        if self.handle is not None:
            self.handle.cancel()
            self.handle = None
