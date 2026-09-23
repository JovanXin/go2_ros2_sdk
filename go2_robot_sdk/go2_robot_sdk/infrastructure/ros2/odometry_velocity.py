"""Measured planar body velocity from successive odometry poses (no commands)."""
from collections import deque
import math


class OdometryVelocity:
    """Short differentiation window reduces network arrival jitter at low cost.

    Call with a consistent measurement clock. A gap, backwards clock, invalid
    quaternion or implausible pose jump resets history instead of inventing a
    large velocity. None means insufficient/invalid feedback, not measured rest.
    """

    def __init__(self, window=0.1, max_gap=0.5):
        self.window = window
        self.max_gap = max_gap
        self.samples = deque()

    def update(self, stamp, position, orientation):
        x, y = float(position['x']), float(position['y'])
        qx, qy, qz, qw = (float(orientation[k]) for k in ('x', 'y', 'z', 'w'))
        norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if not all(math.isfinite(v) for v in (stamp, x, y, norm)) or norm < 1e-6:
            self.samples.clear()
            return None
        qx, qy, qz, qw = (v / norm for v in (qx, qy, qz, qw))
        yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
        sample = (stamp, x, y, yaw)
        if self.samples:
            previous = self.samples[-1]
            dt = stamp - previous[0]
            if dt == 0:
                return None
            if dt < 0 or dt > self.max_gap:
                self.samples.clear()
            elif (math.hypot(x-previous[1], y-previous[2]) / dt > 5.0 or
                  abs(math.remainder(yaw-previous[3], 2*math.pi)) / dt > 10.0):
                self.samples.clear()
        self.samples.append(sample)
        while len(self.samples) > 2 and self.samples[1][0] <= stamp - self.window + 1e-6:
            self.samples.popleft()
        first = self.samples[0]
        dt = stamp - first[0]
        if dt < 0.02:
            return None
        vx, vy = (x-first[1]) / dt, (y-first[2]) / dt
        return (math.cos(yaw)*vx + math.sin(yaw)*vy,
                -math.sin(yaw)*vx + math.cos(yaw)*vy,
                math.remainder(yaw-first[3], 2*math.pi) / dt)
