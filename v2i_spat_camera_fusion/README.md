# Camera/SPaT fusion field tests

Build from the Autoware workspace and source the resulting overlay:

```bash
colcon build --packages-select v2i_spat_fusion_debug_msgs v2i_spat_camera_fusion --symlink-install
source install/setup.bash
```

Start the complete hybrid pipeline with 400 ms of added SPaT delay:

```bash
ros2 launch v2i_spat_camera_fusion v2i_spat_camera_fusion_field_test.launch.py delay_ms:=400.0
```

This includes the normal hybrid launch (OBU bridge, SPaT status publisher,
camera calibration TFs, CameraInfo republisher, map-based detector, HSV
classifier and fusion) plus the delay node. Existing camera, map and localization
inputs must be supplied as for the normal hybrid launch. Run this instead of
another instance of the normal fusion launch to avoid duplicate publishers.
`obu_bridge_node:=false` disables the bridge when SPaT packets are supplied
elsewhere. `use_sim_time:=true` uses ROS /clock for replay and delay scheduling.

Pipeline:

- Original SPaT: `/v2i/spat/traffic_signals`
- Delay node output / fusion SPaT input: `/v2i/spat/traffic_signals_delayed`
- Camera input: `/traffic_light_hsv_roi_classifier/output/traffic_signals`
- Fused output: `/perception/traffic_light_recognition/traffic_signals`
- Diagnostics: `/v2i_spat_camera_fusion/debug/fusion_diagnostics`

`delay_ms` defaults to 0.0 (reference run). Original message stamps and content
are preserved. A nonblocking FIFO releases messages after the requested delay;
the default 5 ms timer and executor/transport scheduling can add extra latency.
The original stamp may already be older than the delay-node arrival time.
Messages due at shutdown are not flushed; allow the queue to drain before
stopping recording. A backwards ROS-clock jump clears the old timeline queue.

For independent constant-delay runs, choose delay_ms at startup. To change it
manually during a run:

```bash
ros2 param set /spat_delay_node delay_ms 600.0
```

Changes apply to new arrivals only. Already queued messages keep their deadlines.
FIFO is preserved when reducing delay, so new messages can wait behind old ones
and then be released together. Exclude this transition from steady-state rate
measurements. Negative and nonfinite delays are rejected. Restart to change
input_topic, output_topic or timer_period_ms.

The normal `v2i_spat_camera_fusion.launch.py` still defaults to direct SPaT input
and method=v2i. Its optional `fusion_spat_input_topic` argument configures only
the hybrid fusion subscriber. Fusion decision rules and reliability calculations
are unchanged by the field-test launcher.

Run queue/timing tests:

```bash
colcon test --packages-select v2i_spat_camera_fusion
colcon test-result --verbose
```
