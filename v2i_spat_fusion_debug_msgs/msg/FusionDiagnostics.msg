# Same stamp as the corresponding fused TrafficLightGroupArray.
builtin_interfaces/Time stamp
uint64 cycle_id
# Sampling time for the source-level timing observations below.
builtin_interfaces/Time sampled_at
bool camera_received
bool spat_received
builtin_interfaces/Time camera_stamp
builtin_interfaces/Time spat_stamp
bool camera_stamp_valid
bool spat_stamp_valid
# Seconds; -1 means unavailable. Message age uses the algorithm's zero clamp.
float64 camera_message_age
float64 spat_message_age
float64 camera_time_since_arrival
float64 spat_time_since_arrival
# Hz measured from up to ten arrivals, not a wall-clock sliding window.
float64 camera_update_rate_hz
float64 spat_update_rate_hz
uint32 camera_history_samples
uint32 spat_history_samples
float64 camera_max_latency
float64 spat_max_latency
float64 camera_expected_rate
float64 spat_expected_rate
float64 camera_reliability_threshold
float64 spat_reliability_threshold
FusionGroupDiagnostics[] groups
