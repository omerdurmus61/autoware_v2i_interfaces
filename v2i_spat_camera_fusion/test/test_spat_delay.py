"""Deterministic checks for timing, FIFO, message integrity and clock resets."""

from collections import deque
import importlib.util
from pathlib import Path
from types import SimpleNamespace

from autoware_perception_msgs.msg import TrafficLightGroup, TrafficLightGroupArray
from rclpy.parameter import Parameter


spec = importlib.util.spec_from_file_location(
    "spat_delay", Path(__file__).resolve().parents[1] / "scripts/spat_delay_node.py"
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def make_node(delay_ms):
    node = module.SpatDelayNode.__new__(module.SpatDelayNode)
    node.pending = deque()
    node.last_clock_ns = None
    state = SimpleNamespace(now_ns=1_000_000_000, delay_ms=delay_ms, output=[])
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=state.now_ns)
    )
    node.get_parameter = lambda name: SimpleNamespace(value=state.delay_ms)
    node.get_logger = lambda: SimpleNamespace(warn=lambda message: None)
    node.publisher = SimpleNamespace(publish=state.output.append)
    return node, state


def message(identifier):
    msg = TrafficLightGroupArray()
    msg.stamp.sec = 1
    group = TrafficLightGroup()
    group.traffic_light_group_id = identifier
    msg.traffic_light_groups = [group]
    return msg


def test_delay_preserves_stamp_contents_order_and_rate():
    node, state = make_node(400.0)
    originals = [message(i) for i in range(5)]
    for i, msg in enumerate(originals):
        state.now_ns = 1_000_000_000 + i * 100_000_000
        node.receive(msg)
    assert state.output == originals[:1]
    for i in range(1, 5):
        state.now_ns = 1_400_000_000 + i * 100_000_000
        node.release_due()
        assert state.output == originals[:i + 1]
    assert not node.pending


def test_no_early_release_and_input_is_copied():
    node, state = make_node(200.0)
    msg = message(42)
    node.receive(msg)
    msg.traffic_light_groups.clear()
    state.now_ns += 199_999_999
    node.release_due()
    assert not state.output
    state.now_ns += 1
    node.release_due()
    assert state.output == [message(42)]


def test_zero_delay_passes_through():
    node, state = make_node(0.0)
    msg = message(42)
    node.receive(msg)
    assert state.output == [msg]
    assert not node.pending


def test_runtime_delay_reduction_preserves_fifo():
    node, state = make_node(500.0)
    node.receive(message(1))
    state.now_ns += 100_000_000
    state.delay_ms = 0.0
    node.receive(message(2))
    assert not state.output
    state.now_ns = 1_500_000_000
    node.release_due()
    assert state.output == [message(1), message(2)]


def test_clock_reset_discards_old_timeline():
    node, state = make_node(500.0)
    node.receive(message(1))
    state.now_ns = 0
    node.release_due()
    assert not node.pending
    assert not state.output
    node.receive(message(2))
    state.now_ns = 500_000_000
    node.release_due()
    assert state.output == [message(2)]


def test_parameter_validation_has_no_side_effects():
    node, state = make_node(400.0)
    for invalid in [-1.0, float('nan'), float('inf')]:
        result = node.validate_parameters([Parameter('delay_ms', value=invalid)])
        assert not result.successful
    assert node.validate_parameters([Parameter('delay_ms', value=600.0)]).successful
    assert not node.validate_parameters([Parameter('input_topic', value='/other')]).successful
    assert state.delay_ms == 400.0
