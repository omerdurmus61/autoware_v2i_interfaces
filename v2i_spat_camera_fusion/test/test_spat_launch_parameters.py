"""Check that mode-specific routing cannot conflict with named YAML values."""

import importlib.util
from pathlib import Path

from launch.actions import GroupAction
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
from launch import LaunchContext
import yaml


spec = importlib.util.spec_from_file_location(
    "fusion_launch",
    Path(__file__).resolve().parents[1] / "launch/v2i_spat_camera_fusion.launch.py",
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_status_output_override_preserves_other_settings(tmp_path):
    config = tmp_path / "status.yaml"
    settings = {
        "input_topic": "/v2i/spat/raw",
        "output_topic": module.AUTOWARE_TRAFFIC_SIGNALS_TOPIC,
        "publish_rate_hz": 10.0,
        "signal_timeout_sec": 0.75,
        "keep_last_valid_on_unknown": True,
    }
    config.write_text(yaml.safe_dump({
        "v2i_traffic_light_status_publisher": {"ros__parameters": settings}
    }))
    for output in (module.SPAT_TRAFFIC_SIGNALS_TOPIC, module.AUTOWARE_TRAFFIC_SIGNALS_TOPIC):
        result = module._status_publisher_parameters(config, output)
        assert result == dict(settings, output_topic=output)


def test_launch_uses_one_parameter_dictionary_for_each_status_publisher():
    description = module.generate_launch_description()
    status_nodes = [
        action
        for group in description.entities if isinstance(group, GroupAction)
        for action in group.get_sub_entities()
        if isinstance(action, Node) and action.node_package == "v2i_traffic_light_status_publisher"
    ]
    assert len(status_nodes) == 2
    outputs = []
    for node in status_nodes:
        parameters = evaluate_parameters(LaunchContext(), node._Node__parameters)
        assert len(parameters) == 1
        assert isinstance(parameters[0], dict)
        outputs.append(parameters[0]["output_topic"])
    assert outputs == [module.AUTOWARE_TRAFFIC_SIGNALS_TOPIC, module.SPAT_TRAFFIC_SIGNALS_TOPIC]
