uint8 OBJECT_TYPE_UNKNOWN=0
uint8 OBJECT_TYPE_VEHICLE=1
uint8 OBJECT_TYPE_VRU=2

uint8 object_type
uint16 object_id
uint8 object_type_confidence

uint16 measurement_time
string time_confidence

SDSMPosition position

string position_confidence
uint16 speed
string speed_confidence
uint16 heading
string heading_confidence

uint16 width
uint16 length

string vru_basic_type
string vru_propulsion
uint16 vru_radius