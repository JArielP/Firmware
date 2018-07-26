uint32 MODE_NONE             = 1
uint32 MODE_FIXED_PITCH      = 2
uint32 MODE_FIXED_PITCH_2    = 4
uint32 MODE_FIXED_PITCH_3    = 8
uint32 MODE_PITCH_RAMP       = 16
uint32 MODE_FIXED_ELEVATOR   = 32
uint32 MODE_211_PITCH        = 64
uint32 MODE_211_ROLL         = 128
uint32 MODE_211_YAW          = 256
uint32 MODE_MANUAL_ELEVATOR  = 512
uint32 MODE_MAX              = 1024


uint16 mode                         # describes in witch sys_id maneuver (according to MODE bitmask) we are currently in
uint32 reset_count                  # describes how often the pilot switched into the sys_id maneuver mode in one flight.
uint64 timestamp_start_maneuver     # starting time of the current maneuver
bool   maneuver_valid               # true if current maneuver (or last, if in recover-phase) is valid.