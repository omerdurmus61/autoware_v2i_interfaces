# One element of 'state-time-speed' list

uint8 event_state
TimeChangeDetails timing

# EventState enum mapping (based on observed strings)
uint8 EVENT_UNKNOWN=0
uint8 STOP_AND_REMAIN=1
uint8 PROTECTED_MOVEMENT_ALLOWED=2
uint8 PERMISSIVE_CLEARANCE=3
uint8 PERMISSIVE_MOVEMENT_ALLOWED=4
