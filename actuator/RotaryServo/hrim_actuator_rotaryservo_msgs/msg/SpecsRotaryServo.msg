std_msgs/Header header

# type of control that accepts the rotary servomotor
uint8 CONTROL_TYPE_NONE=0
uint8 CONTROL_TYPE_POSITION=1
uint8 CONTROL_TYPE_EFFORT=2
uint8 CONTROL_TYPE_VELOCITY=3
uint8 CONTROL_TYPE_POSITION_VELOCITY=4
uint8 CONTROL_TYPE_POSITION_EFFORT=5
uint8 CONTROL_TYPE_VELOCITY_EFFORT=6
uint8 CONTROL_TYPE_POSITION_VELOCITY_EFFORT=7
uint8 control_type                               # rotary servomotor control type

float64 range_min              # minimum work range (rad)
float64 range_max              # maximum work range (rad)
float64 precision              # angular precision (rad)
float64 rated_speed            # servomotor speed (rad/s)
float64 reachable_speed        # maximum speed (rad/s)
float64 rated_torque           # servomotor torque (Nm)
float64 reachable_torque       # peak torque (Nm)
float64 temperature_range_min  # minimum operational temperature (celsius)
float64 temperature_range_max  # maximum operational temperature (celsius)
