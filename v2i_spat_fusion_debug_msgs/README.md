# Fusion diagnostics

The `v2i_spat_camera_fusion` fusion node publishes `FusionDiagnostics` on
`/v2i_spat_camera_fusion/debug/fusion_diagnostics` at its fusion timer rate
(default 10 Hz). Override the node parameter `debug_diagnostics_topic` to change
this topic. QoS is reliable, volatile, keep-last depth 1.

`stamp` matches the fused `TrafficLightGroupArray`; `cycle_id` starts at zero
and increases each cycle, including cycles with no groups. `groups` contains
one record per fused group, including the exact scores, predicates, selected
source and state used by that decision. State strings are RED, AMBER, GREEN,
and UNKNOWN. Source and reason constants are defined in the group message.
`comparison_valid` excludes missing groups and UNKNOWN states; disagreement
is false when comparison is invalid.

Source-level timing observations are sampled after the decision at `sampled_at`.
They can differ slightly from the time used for per-group freshness. Ages and
arrival intervals are in seconds; unavailable observations use -1 and explicit
validity/presence flags. A zero timestamp is invalid according to the existing
fusion algorithm. Future timestamps are clamped to zero age. All times use the
node ROS clock, including simulation time when enabled.

Message age describes the received message stamp, not necessarily the original
camera acquisition or roadside SPaT generation time: upstream publishers may
restamp cached data. Arrival rate uses the last ten arrivals and does not decay
when input stops; time-since-arrival exposes such stops. With fewer than two
arrivals the algorithm's rate score is 1 even though measured rate is 0 Hz.

The existing algorithm is preserved: output confidence is the maximum of the
two reliability values (zero for UNKNOWN), not necessarily the reliability of
the selected source. V2I can be selected with an UNKNOWN state. The legacy
String source topic remains available but does not describe per-group choices.
Record the diagnostics together with both inputs, fused output, and an
independent ground truth for accuracy experiments. This package adds no delay
or decision-policy changes.
