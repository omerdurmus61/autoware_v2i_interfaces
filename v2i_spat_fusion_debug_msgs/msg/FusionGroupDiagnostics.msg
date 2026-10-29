# Values captured directly from the decision for this group.
string SOURCE_V2I=v2i
string SOURCE_CAMERA=camera
string SOURCE_UNKNOWN=unknown
string REASON_SPAT_ACCEPTED=SPAT_ACCEPTED
string REASON_CAMERA_FALLBACK=CAMERA_FALLBACK
string REASON_NO_USABLE_SOURCE=NO_USABLE_SOURCE
int64 traffic_light_group_id
bool camera_present
bool spat_present
# States: RED, AMBER, GREEN, UNKNOWN.
string camera_state
string spat_state
float64 camera_confidence
float64 camera_freshness
float64 spat_freshness
float64 camera_update_rate_score
float64 spat_update_rate_score
float64 id_match_score
float64 camera_reliability
float64 spat_reliability
bool camera_reliable
bool spat_reliable
bool spat_fresh
string selected_source
string selected_state
string decision_reason
float64 output_confidence
# Disagreement is meaningful only if both groups have known states.
bool comparison_valid
bool camera_spat_disagree
