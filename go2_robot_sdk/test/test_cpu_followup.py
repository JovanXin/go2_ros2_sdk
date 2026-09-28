"""Offline contracts: no SDK imports, ROS participant, network or robot motion."""
import ast
import asyncio
import heapq
import logging
from pathlib import Path
import queue
import runpy
import threading
from types import SimpleNamespace as NS
from typing import Callable
from unittest.mock import Mock

import pytest

ROOT = Path(__file__).resolve().parents[1] / 'go2_robot_sdk'
telemetry = runpy.run_path(str(ROOT / 'infrastructure/ros2/telemetry_buffer.py'))
LatestTelemetry = telemetry['LatestTelemetry']
TelemetryWakeup = telemetry['TelemetryWakeup']


def load_class(path, name, **namespace):
    tree = ast.parse((ROOT / path).read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == name)
    cls.bases = []
    ns = dict(asyncio=asyncio, queue=queue, logger=logging.getLogger(__name__),
              Dict=dict, Any=object, Callable=Callable, RobotConfig=object,
              RobotData=NS, IRobotDataPublisher=object, **namespace)
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(path), 'exec'), ns)
    return ns[name]


def adapter():
    cls = load_class('infrastructure/webrtc/webrtc_adapter.py', 'WebRTCAdapter',
                     gen_command=lambda api_id, parameter, topic, command_id=None: parameter)
    return cls(NS(), None)


def test_threaded_fifo_stop_order_and_accounting():
    a = adapter()
    sent = []
    a.send_command = lambda robot, msg: sent.append(msg)
    expected = list(range(10000)) + ['STOP', 'configuration']
    def produce():
        for item in expected:
            a.send_webrtc_request('0', 1, item, 'request')
    producer = threading.Thread(target=produce)
    producer.start()
    while producer.is_alive() or not a.webrtc_msgs.empty():
        a.process_webrtc_commands('0')
    producer.join()
    assert sent == expected
    assert a.webrtc_msgs.unfinished_tasks == 0
    a.webrtc_msgs.join()
    assert adapter().webrtc_msgs.empty()  # process restart cannot replay commands


def test_insertion_and_accounting_are_atomic_to_consumer():
    a = adapter()
    inserted, attempting, consumed = (threading.Event() for _ in range(3))
    original_put = a.webrtc_msgs._put
    def paused_put(item):
        original_put(item)  # same insertion/accounting boundary as audit reproduction
        inserted.set()
        assert attempting.wait(2)
        assert not consumed.wait(.02)  # consumer cannot acquire the Queue mutex
    a.webrtc_msgs._put = paused_put
    a.send_command = lambda *args: consumed.set()
    def consume():
        assert inserted.wait(2)
        attempting.set()
        a.process_webrtc_commands('0')
    consumer = threading.Thread(target=consume)
    consumer.start()
    a.send_webrtc_request('0', 1, 'STOP', 'request')
    consumer.join(2)
    assert not consumer.is_alive()
    assert consumed.is_set()
    assert a.webrtc_msgs.unfinished_tasks == 0


def test_drain_is_bounded_even_when_send_replenishes_queue():
    a = adapter()
    def send(*args):
        a.webrtc_msgs.put_nowait('next')
    a.send_command = Mock(side_effect=send)
    a.webrtc_msgs.put_nowait('move')
    a.process_webrtc_commands('0')
    assert a.send_command.call_count == a.MAX_COMMANDS_PER_DRAIN
    assert a.webrtc_msgs.qsize() == a.webrtc_msgs.unfinished_tasks == 1


def test_send_failure_balances_accounting_and_preserves_following_stop():
    a = adapter()
    a.send_command = Mock(side_effect=RuntimeError('link down'))
    a.webrtc_msgs.put_nowait('move')
    a.webrtc_msgs.put_nowait('STOP')
    with pytest.raises(RuntimeError):
        a.process_webrtc_commands('0')
    assert a.webrtc_msgs.unfinished_tasks == 1
    a.send_command = Mock()
    a.process_webrtc_commands('0')
    a.send_command.assert_called_once_with('0', 'STOP')
    assert a.webrtc_msgs.unfinished_tasks == 0


class FakeLoop:
    def __init__(self):
        self.now = 1.0
        self.events = []
        self.serial = 0

    def call_later(self, delay, callback):
        handle = NS(cancelled=False)
        handle.cancel = lambda: setattr(handle, 'cancelled', True)
        self.serial += 1
        heapq.heappush(self.events, (self.now + delay, self.serial, handle, callback))
        return handle

    def advance(self, until):
        while self.events and self.events[0][0] <= until:
            self.now, _, handle, callback = heapq.heappop(self.events)
            if not handle.cancelled:
                callback()
        self.now = until


def scheduler(rates=None):
    loop = FakeLoop()
    buffer = LatestTelemetry(rates or {'pose': 20.0})
    wake = Mock()
    clock = lambda: loop.now
    s = TelemetryWakeup(buffer, loop, wake, clock)
    def put(topic, value):
        if buffer.put('0', topic, value):
            s.received()
    def drain():
        result = buffer.take_ready(loop.now)
        s.published()
        return result
    return loop, buffer, wake, s, put, drain


def test_final_early_sample_gets_due_timer_and_idle_never_wakes():
    loop, buffer, wake, s, put, drain = scheduler()
    put('pose', 'first')
    loop.advance(1.)
    assert drain() == [('0', 'first')]
    loop.advance(1.01)
    put('pose', 'final')
    loop.advance(1.049)
    assert wake.call_count == 1
    loop.advance(1.05)
    assert wake.call_count == 2
    assert drain() == [('0', 'final')]
    loop.advance(100.)
    assert wake.call_count == 2
    assert buffer.earliest_due() is None


def test_fast_source_is_rate_limited_without_poll_quantization():
    loop, buffer, wake, s, put, drain = scheduler()
    publications = []
    def publish():
        publications.append((loop.now, drain()))
    wake.side_effect = publish
    for i in range(200):  # 200 Hz source, 20 Hz ROS output
        loop.advance(1.0 + i * .005)
        put('pose', i)
    loop.advance(2.001)
    assert 20 <= len(publications) <= 21
    assert all(b[0] - a[0] >= .05 - 1e-9
               for a, b in zip(publications, publications[1:]))
    assert publications[-1][1] == [('0', 199)]
    assert all(len(samples) == 1 for _, samples in publications)


def test_new_topic_advances_deadline_and_inflight_coalesces_arrivals():
    loop, buffer, wake, s, put, drain = scheduler({'pose': 20., 'battery': 5.})
    put('battery', 90)
    loop.advance(1.)
    drain()
    put('battery', 89)  # due 1.2
    loop.advance(1.01)
    put('pose', 1)  # due now, cannot wait behind battery
    loop.advance(1.01)
    for i in range(1000):
        put('pose', i)
    loop.advance(1.1)  # simulate a delayed executor, only one guard in flight
    assert wake.call_count == 2
    assert drain() == [('0', 999)]
    loop.advance(1.2)
    assert drain() == [('0', 89)]


def test_arrival_between_ros_drain_and_loop_ack_is_not_stranded():
    loop, buffer, wake, s, put, drain = scheduler()
    put('pose', 1)
    loop.advance(1.)
    assert buffer.take_ready(loop.now) == [('0', 1)]
    put('pose', 2)
    s.published()
    loop.advance(1.05)
    assert drain() == [('0', 2)]


def test_shutdown_cancels_pending_timer_and_ignores_late_ack():
    loop, buffer, wake, s, put, drain = scheduler()
    put('pose', 1)
    s.close()
    s.published()
    put('pose', 2)
    loop.advance(2.)
    wake.assert_not_called()


def test_joint_opt_out_skips_processing_without_changing_other_topics():
    topics = {key: key for key in ('ULIDAR_ARRAY', 'ROBOTODOM', 'LF_SPORT_MOD_STATE', 'LOW_STATE')}
    cls = load_class('application/services/robot_data_service.py', 'RobotDataService',
                     RTC_TOPIC=topics)
    for enabled in (False, True):
        publisher = Mock()
        service = cls(publisher, publish_joint_states=enabled)
        service._process_low_state = Mock()
        service.process_webrtc_message({'topic': 'LOW_STATE'}, '0')
        assert service._process_low_state.call_count == int(enabled)
        assert publisher.publish_joint_state.call_count == int(enabled)
        service._process_odometry_data = Mock()
        service.process_webrtc_message({'topic': 'ROBOTODOM'}, '0')
        publisher.publish_odometry.assert_called_once()


def test_battery_is_published_before_optional_joint_processing():
    # Execute actual driver methods without importing ROS or aiortc.
    tree = ast.parse((ROOT / 'presentation/go2_driver_node.py').read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in
               ('_publish_latest_telemetry', '_publish_battery_telemetry')]
    ns = dict(time=NS(monotonic=lambda: 1.), RTC_TOPIC={'LOW_STATE': 'low'},
              BatteryState=lambda: NS(header=NS()))
    exec(compile(ast.Module(body=methods, type_ignores=[]), '<driver>', 'exec'), ns)
    buffer = LatestTelemetry({'low': 5.})
    buffer.put('0', 'low', {'topic': 'low', 'data': {'bms_state': {'soc': 83}}})
    node = NS(_latest_telemetry=buffer, robot_data_service=Mock(), event_loop=Mock(),
              _telemetry_wakeup=Mock(), publishers_dict={'battery': [Mock()]},
              get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: 'stamp')))
    node._publish_battery_telemetry = lambda msg, robot: ns['_publish_battery_telemetry'](node, msg, robot)
    ns['_publish_latest_telemetry'](node)
    battery = node.publishers_dict['battery'][0].publish.call_args.args[0]
    assert battery.percentage == .83
    assert battery.present
    node.event_loop.call_soon_threadsafe.assert_called_once_with(node._telemetry_wakeup.published)
