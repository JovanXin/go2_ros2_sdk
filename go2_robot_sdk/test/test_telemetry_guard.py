"""Local ROS guard integration only: no publishers, subscriptions or robot link."""
import asyncio
from pathlib import Path
import runpy
import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

module = runpy.run_path(str(Path(__file__).resolve().parents[1] /
                           'go2_robot_sdk/infrastructure/ros2/telemetry_buffer.py'))


def test_asyncio_deadline_wakes_ros_executor_and_delivers_final_sample():
    async def exercise():
        # Separate participant domain; no command entities even in this domain.
        context = Context()
        context.init(args=[], domain_id=187)
        node = Node('telemetry_guard_offline_test', context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        loop = asyncio.get_running_loop()
        buffer = module['LatestTelemetry']({'pose': 20.})
        received = asyncio.Queue()
        callbacks = []
        def publish():
            callbacks.append(threading.get_ident())
            for sample in buffer.take_ready(time.monotonic()):
                loop.call_soon_threadsafe(received.put_nowait, sample)
            loop.call_soon_threadsafe(scheduler.published)
        guard = node.create_guard_condition(publish)
        scheduler = module['TelemetryWakeup'](buffer, loop, guard.trigger)
        thread = threading.Thread(target=executor.spin)
        thread.start()
        try:
            buffer.put('0', 'pose', {'stamp': 100, 'pose': 1})
            scheduler.received()
            assert await asyncio.wait_for(received.get(), 2) == (
                '0', {'stamp': 100, 'pose': 1})
            # No further source packet after this early sample. It must not wait
            # for another arrival, and its source stamp must pass through intact.
            buffer.put('0', 'pose', {'stamp': 101, 'pose': 2})
            scheduler.received()
            assert await asyncio.wait_for(received.get(), 2) == (
                '0', {'stamp': 101, 'pose': 2})
            await asyncio.sleep(.1)
            assert len(callbacks) == 2
            assert all(tid == thread.ident for tid in callbacks)
            assert received.empty()
        finally:
            scheduler.close()
            executor.shutdown(timeout_sec=2)
            thread.join(2)
            node.destroy_node()
            context.shutdown()
        assert not thread.is_alive()
    asyncio.run(exercise())
