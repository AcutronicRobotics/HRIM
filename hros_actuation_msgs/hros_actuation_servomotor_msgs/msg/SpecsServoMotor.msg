std_msgs/Header header

string joint_name

# type of control that accepts the rotary servomotor
uint8 CONTROL_TYPE_POSITION=0
uint8 CONTROL_TYPE_POSITION_VELOCITY=1
uint8 CONTROL_TYPE_POSITION_VELOCITY_EFFORT=0
uint8 control_type                               # servomotor control type

float64 range_min              # servomotor work range max (rad)
float64 range_max              # servomotor work range min (rad)
float64 precision              # angular precision (rad)
float64 rated_speed            # servomotor speed (rad/s)
float64 reachable_speed        # maximum speed (rad/s)
float64 rated_torque           # servomotor torque (Nm)
float64 reachable_torque       # peak torque (Nm)
float64 temperature_range_min  # operational temperature range (celsius)
float64 temperature_range_max  # operational temperature range (celsius)
