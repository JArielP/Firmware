uint8 MODE_NONE             = 1
uint8 MODE_FIXED_PITCH      = 2
uint8 MODE_FIXED_ELEVATOR   = 4
uint8 MODE_211_ROLL         = 8
uint8 MODE_211_PITCH        = 16
uint8 MODE_211_YAW          = 32
uint8 MODE_MAX              = 64

uint8 mode                          # describes in witch sys_id maneuver (according to MODE bitmask) we are currently in
uint64 timestamp_start_maneuver     # starting time of the current maneuver
bool maneuver_valid                 # true if current maneuver (or last, if in recover-phase) is valid.