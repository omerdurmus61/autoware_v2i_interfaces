# V2I SPaT RViz Panel

This package provides an RViz panel for V2I SPaT traffic signal groups.

The panel subscribes to `/v2i/spat/raw`, displays the configured signal groups, and shows the current signal state and estimated remaining time. A signal is marked as `STALE` when no SPaT update has been received for 1.5 seconds.

## Build

Build the package from the ROS 2 workspace root:

```bash
colcon build --packages-select v2i_spat_rviz_panel
source install/setup.bash
```

## Use

Start the SPAT UDP bridge, start RViz, and add the panel through:

`Panels -> Add New Panel -> v2i_spat_rviz_panel/SpatPanel`

The panel defaults to `/v2i/spat/raw`. The topic is saved in the RViz configuration and can be changed by editing the panel configuration.

The default signal list mirrors `v2i_spat_visualizer/config/intersections.yaml`.