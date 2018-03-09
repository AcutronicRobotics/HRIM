std_msgs/Header header

uint8 control_type             # servomotor control type, explained in SpecsRotaryServo.msg

float64 position               # Position to move (rad)
float32 velocity               # Velocity to move (rad/s)
float32 effort                 # Force to move (kg*cm3)
