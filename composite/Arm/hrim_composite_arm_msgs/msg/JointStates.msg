Header header

uint8 control_type           # arm control type, explained in MoveCommand.srv

string[] joint_names         # parameter that each servomotor has
float64[] position           # servomotor position (rad)
float64[] velocity           # servomotor position (rad/s)
float64[] effort             # servomotor torque (Nm)
