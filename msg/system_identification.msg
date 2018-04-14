uint8 MODE_WAIT_FOR_COMMAND = 0     # this will execute a loiter with radius 30, altitude 50 above home position
uint8 MODE_FLY_TO_HOME      = 1     # flys back to home position to eventually restart a maneuver.

uint8 MODE_STABILIZED       = 10
uint8 MODE_FIXED            = 20

uint8 mode
uint64 timestamp_start              # starting time of the current maneuver
bool in_maneuver                    # true when vehicle is currently in sys_id maneuver (and not preparing for it)