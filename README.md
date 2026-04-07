# V2I Packages for ROS 2 + Autoware

This workspace contains ROS 2 packages that decode V2I UDP traffic (SPaT, MAP, SDSM), convert it into custom ROS messages, and adapt it to Autoware messages.

## Package Overview

| Package | Type | Purpose | Main Input | Main Output |
|---|---|---|---|---|
| `v2i_spat_bridge` | Bridge node | Decode SPaT UDP packets into ROS | UDP `:7114` | `/v2i/spat/raw` (`v2i_spat_msgs/SpatPacket`) |
| `v2i_traffic_light_status_publisher` | Converter node | Convert SPaT to Autoware traffic lights | `/v2i/spat/raw` | `/perception/traffic_light_recognition/traffic_signals` |
| `v2i_map_bridge` | Bridge node | Decode MAP UDP packets into ROS | UDP `:7113` | `/v2i/map/raw` (`v2i_map_msgs/MapData`) |
| `v2i_map_to_rviz_markers` | Visualization node | Convert MAP to RViz markers | `/v2i/map/raw` | `/v2i/map/lane_markers` (`visualization_msgs/MarkerArray`) |
| `v2i_sdsm_bridge` | Bridge node | Decode SDSM UDP packets into ROS | UDP `:7112` | `/v2i/sdsm/raw` (`v2i_sdsm_msgs/SDSM`) |
| `v2i_sdsm_to_autoware_objects` | Converter node | Convert SDSM to Autoware detected objects | `/v2i/sdsm/raw` | `/v2i/sdsm/objects` (`autoware_perception_msgs/DetectedObjects`) |
| `v2i_spat_msgs` | Message package | SPaT message definitions | N/A | Custom message types |
| `v2i_map_msgs` | Message package | MAP message definitions | N/A | Custom message types |
| `v2i_sdsm_msgs` | Message package | SDSM message definitions | N/A | Custom message types |

---

## Quick Start

### 1) Build

```bash
cd ~/v2i_tools_ws
source /opt/ros/<ros_distro>/setup.bash
colcon build --packages-select \
  v2i_spat_msgs v2i_map_msgs v2i_sdsm_msgs \
  v2i_spat_bridge v2i_traffic_light_status_publisher \
  v2i_map_bridge v2i_map_to_rviz_markers \
  v2i_sdsm_bridge v2i_sdsm_to_autoware_objects
source install/setup.bash
```

### 2) Run a full SPaT pipeline (example)

Terminal 1:
```bash
ros2 launch v2i_spat_bridge spat_udp_bridge.launch.py
```

Terminal 2:
```bash
ros2 launch v2i_traffic_light_status_publisher v2i_traffic_light_status_publisher.launch.py
```

Terminal 3:
```bash
ros2 topic echo /perception/traffic_light_recognition/traffic_signals
```

---

## Package-by-Package Guide

### `v2i_spat_bridge`
Brief: Listens for SPaT UDP frames, ASN.1-decodes them, and publishes structured SPaT ROS messages.

Example launch:
```bash
ros2 launch v2i_spat_bridge spat_udp_bridge.launch.py
```

Example override (port/topic):
```bash
ros2 run v2i_spat_bridge spat_udp_bridge.py --ros-args \
  -p udp_port:=7114 \
  -p topic:=/v2i/spat/raw
```

Useful params (`config/spat_udp_bridge.yaml`):
- `bind_ip` (default: `0.0.0.0`)
- `udp_port` (default: `7114`)
- `filter_hex_prefix` (default: `0013`)
- `topic` (default: `/v2i/spat/raw`)

### `v2i_traffic_light_status_publisher`
Brief: Subscribes to SPaT and publishes Autoware `TrafficLightGroupArray` for traffic light perception/planning.

Example launch:
```bash
ros2 launch v2i_traffic_light_status_publisher v2i_traffic_light_status_publisher.launch.py
```

Example output check:
```bash
ros2 topic echo /perception/traffic_light_recognition/traffic_signals
```

Useful params (`config/v2i_traffic_light_status_publisher.yaml`):
- `input_topic` (default: `/v2i/spat/raw`)
- `output_topic` (default: `/perception/traffic_light_recognition/traffic_signals`)
- `publish_rate_hz` (default: `10.0`)
- `signal_timeout_sec` (default: `0.75`)

### `v2i_map_bridge`
Brief: Listens for MAP UDP frames, decodes lane/intersection geometry, and publishes `v2i_map_msgs/MapData`.

Example launch:
```bash
ros2 launch v2i_map_bridge map_udp_bridge.launch.py
```

Example output check:
```bash
ros2 topic echo /v2i/map/raw
```

Useful params (`config/map_udp_bridge.yaml`):
- `udp_port` (default: `7113`)
- `filter_hex_prefix` (default: `0012`)
- `topic` (default: `/v2i/map/raw`)

### `v2i_map_to_rviz_markers`
Brief: Converts `MapData` into RViz marker layers (lane lines, IDs, arrows, connections).

Example launch:
```bash
ros2 launch v2i_map_to_rviz_markers map_to_rviz_markers.launch.py
```

Optional static TF launch:
```bash
ros2 launch v2i_map_to_rviz_markers intersections_tf.launch.py
```

Example RViz topic check:
```bash
ros2 topic echo /v2i/map/lane_markers
```

Useful params (`config/map_to_rviz_markers.yaml`):
- `input_topic` / `output_topic`
- `publish_all_intersections`
- `target_intersection_id`
- `node_scale_x`, `node_scale_y`, `swap_xy`, `invert_x`, `invert_y`

### `v2i_sdsm_bridge`
Brief: Listens for SDSM UDP frames and publishes decoded object-level infrastructure detections.

Example launch:
```bash
ros2 launch v2i_sdsm_bridge sdsm_udp_bridge.launch.py
```

Example output check:
```bash
ros2 topic echo /v2i/sdsm/raw
```

Useful params (`config/sdsm_udp_bridge.yaml`):
- `udp_port` (default: `7112`)
- `filter_hex_prefix` (default: `0029`)
- `topic` (default: `/v2i/sdsm/raw`)
- `frame_id`

### `v2i_sdsm_to_autoware_objects`
Brief: Converts SDSM objects into `autoware_perception_msgs/DetectedObjects` with configurable scaling and frame transforms.

Example launch:
```bash
ros2 launch v2i_sdsm_to_autoware_objects sdsm_to_autoware_objects.launch.py
```

Example output check:
```bash
ros2 topic echo /v2i/sdsm/objects
```

Useful params (`config/sdsm_to_autoware_objects.yaml`):
- `input_topic` / `output_topic`
- `frame_id`
- `position_scale_x`, `position_scale_y`
- `swap_xy`, `invert_x`, `invert_y`
- `use_heading`, `heading_scale_rad`

### `v2i_spat_msgs`
Brief: Custom ROS message definitions for SPaT payloads.

Example inspect:
```bash
ros2 interface show v2i_spat_msgs/msg/SpatPacket
ros2 interface show v2i_spat_msgs/msg/MovementEvent
```

Example Python use:
```python
from v2i_spat_msgs.msg import SpatPacket, MovementEvent

msg = SpatPacket()
msg.message_id = 19
state = MovementEvent()
state.event_state = MovementEvent.STOP_AND_REMAIN
```

### `v2i_map_msgs`
Brief: Custom ROS message definitions for MAP intersections/lanes/connections.

Example inspect:
```bash
ros2 interface show v2i_map_msgs/msg/MapData
ros2 interface show v2i_map_msgs/msg/MapLane
```

Example Python use:
```python
from v2i_map_msgs.msg import MapData, MapIntersection

m = MapData()
i = MapIntersection()
i.name = "Main_And_1st"
m.intersections.append(i)
```

### `v2i_sdsm_msgs`
Brief: Custom ROS message definitions for infrastructure-detected objects and timestamps.

Example inspect:
```bash
ros2 interface show v2i_sdsm_msgs/msg/SDSM
ros2 interface show v2i_sdsm_msgs/msg/SDSMDetectedObject
```

Example Python use:
```python
from v2i_sdsm_msgs.msg import SDSM, SDSMDetectedObject

m = SDSM()
o = SDSMDetectedObject()
o.object_type = SDSMDetectedObject.OBJECT_TYPE_VEHICLE
m.objects.append(o)
```

---

## Typical End-to-End Flows

SPaT to Autoware traffic lights:
```text
UDP SPaT -> v2i_spat_bridge -> /v2i/spat/raw -> v2i_traffic_light_status_publisher -> /perception/traffic_light_recognition/traffic_signals
```

SDSM to Autoware detected objects:
```text
UDP SDSM -> v2i_sdsm_bridge -> /v2i/sdsm/raw -> v2i_sdsm_to_autoware_objects -> /v2i/sdsm/objects
```

MAP to RViz markers:
```text
UDP MAP -> v2i_map_bridge -> /v2i/map/raw -> v2i_map_to_rviz_markers -> /v2i/map/lane_markers
```

## Troubleshooting

- No messages on output topics:
  - Check UDP source is sending to the configured port (`7112/7113/7114`).
  - Verify `filter_hex_prefix` is correct for your feed.
- Topics exist but RViz is empty:
  - Confirm TF frames (`map` and `v2i_intersection_<id>`) are available.
  - Launch `intersections_tf.launch.py` if needed.
- Autoware topic mismatch:
  - Override `output_topic` in launch or YAML to match your stack.
