Header header

int32 mode             # Mode in which to command arm

string[]  names        # Joint names order for command

# Fields of commands indexed according to the Joint names vector.
# Command fields required for a desired mode are listed in the comments
float64[] position     # (radians)       Required for POSITION_MODE and IMPEDANCE_MODE
float64[] velocity     # (radians/sec)   Required for VELOCITY_MODE and IMPEDANCE_MODE
float64[] acceleration # (radians/sec^2) Required for                   
float64[] effort       # (newton-meters) Required for TORQUE_MODE

# Modes available to command arm
int32 POSITION_MODE=1
int32 VELOCITY_MODE=2
int32 TORQUE_MODE=3
int32 IMPEDANCE_MODE=4
