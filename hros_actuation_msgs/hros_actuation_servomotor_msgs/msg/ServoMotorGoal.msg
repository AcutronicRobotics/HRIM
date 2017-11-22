std_msgs/Header header

string joint_name

# type of control that accepts the rotary servomotor
uint8 CONTROL_TYPE_NONE=0
uint8 CONTROL_TYPE_POSITION=1
uint8 CONTROL_TYPE_EFFORT=1
uint8 CONTROL_TYPE_VELOCITY=1
uint8 CONTROL_TYPE_POSITION_VELOCITY=2
uint8 CONTROL_TYPE_POSITION_VELOCITY_EFFORT=3
uint8 control_type                               # servomotor control type

float64 position                                 # Position to move (rad)
float32 velocity                                 # Velocity to move (rad/s)
float32 effort                                   # Force to move (kg*cm3)
