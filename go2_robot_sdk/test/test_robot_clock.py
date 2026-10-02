import importlib.util
from pathlib import Path

# Load the module directly: the package __init__ pulls in ROS and WebRTC dependencies.
_spec = importlib.util.spec_from_file_location(
    'robot_clock', Path(__file__).resolve().parents[1] / 'go2_robot_sdk/infrastructure/ros2/robot_clock.py')
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)
RobotClockMapper = _mod.RobotClockMapper


def feed(mapper, robot_stamps, offset, delays):
    return [mapper.to_host(s, s + offset + d) for s, d in zip(robot_stamps, delays)]


def test_robot_clock_behind_host_maps_to_small_ages():
    m = RobotClockMapper()
    stamps = [100.0 + 0.05 * i for i in range(40)]
    delays = [0.02, 0.06, 0.03, 0.15] * 10            # transport jitter
    out = feed(m, stamps, 1.1, delays)                # robot clock 1.1 s behind the host
    ages = [s + 1.1 + d - o for s, d, o in zip(stamps, delays, out)]
    assert all(-1e-9 <= a <= 0.15 - 0.02 + 1e-9 for a in ages[4:])   # extra delay only, never ~1.1 s


def test_never_future_and_spacing_preserved():
    m = RobotClockMapper()
    stamps = [5.0 + 0.1 * i for i in range(30)]
    out = feed(m, stamps, -3.0, [0.05] * 30)          # robot clock ahead of the host
    for s, o in zip(stamps, out):
        assert o <= s - 3.0 + 0.05 + 1e-9             # <= host receive time
    gaps = [b - a for a, b in zip(out, out[1:])]
    assert all(abs(g - 0.1) < 1e-9 for g in gaps)


def test_forward_step_followed_immediately_back_step_fails_safe():
    m = RobotClockMapper(window_s=10.0)
    for i in range(20):
        m.to_host(100.0 + 0.1 * i, 101.0 + 0.1 * i + 0.03)
    # robot clock jumps 5 s forward: mapped stamp is right away no later than receive time
    host = 103.0 + 0.03
    assert abs(m.to_host(107.0, host) - host) < 1e-9
    # robot clock jumps 2 s back: samples look old (refused downstream), never future
    host2 = 103.13
    out = m.to_host(105.1, host2)
    assert out <= host2 and host2 - out > 1.5
    # ...until the window forgets the earlier minimum
    for i in range(1, 120):
        host_i = host2 + 0.1 * i
        out = m.to_host(105.1 + 0.1 * i, host_i)
    assert host_i - out < 1e-6
