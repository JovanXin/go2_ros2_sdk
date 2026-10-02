# SPDX-License-Identifier: BSD-3-Clause
"""Map the robot's odometry clock onto the host ROS clock."""

from collections import deque


class RobotClockMapper:
    """Express robot-clock measurement stamps in host time.

    The Go2 stamps odometry with its own clock, which is not synchronised with the host:
    on the laptop it runs ~1 s behind, on the Pi 0.05-2.2 s (docs/freshness-2.5s-root-cause
    in go2_frontier). Published as-is, every pose looked ~1 s old and Nav2's freshness check
    refused all control.

    offset = min over the last `window_s` of (host receive time - robot stamp) is the clock
    offset plus the smallest transport delay seen. robot stamp + offset therefore
      * keeps the robot's spacing between samples (no restamping on arrival),
      * is never in the host's future,
      * is as old as this sample's extra delay, so downstream freshness limits keep meaning.
    A robot clock step forward is followed at once (a lower sample); a step back makes
    samples look old until the window forgets the earlier minimum (fails safe: stale).
    """

    def __init__(self, window_s: float = 10.0):
        self.window_s = window_s
        self._samples = deque()   # (host_time, offset); offsets strictly increasing

    def to_host(self, robot_stamp: float, host_now: float) -> float:
        offset = host_now - robot_stamp
        while self._samples and self._samples[-1][1] >= offset:
            self._samples.pop()
        self._samples.append((host_now, offset))
        while self._samples[0][0] < host_now - self.window_s:
            self._samples.popleft()
        return robot_stamp + self._samples[0][1]
