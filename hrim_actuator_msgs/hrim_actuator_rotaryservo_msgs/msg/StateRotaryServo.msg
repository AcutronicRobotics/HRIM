std_msgs/Header header        # Motor state is at this time

uint32 instance_id

float64 goal                  # commanded position (rad)
float64 position              # current position encoder (rad)
float64 error                 # difference between current and goal positions (rad)
float64 velocity              # current velocity (rad/s)
float64 effort                # current effort of the actuator (Nm)
float64 load                  # load imposed on the motor.
bool moving                   # whether the motor is currently in motion

#Fault states which could cause the driver to be immobilized.

uint8 FAULT_NONE=0
uint8 FAULT_CURRENT=1
uint8 FAULT_TEMPERATURE=2
uint8 FAULT_BUS_VOLTAGE=3
uint8 FAULT_BRIDGE_DRIVER=4
uint8 fault # fault cause

# type of control that accepts the rotary servomotor
uint8 CONTROL_TYPE_NONE=0
uint8 CONTROL_TYPE_POSITION=1
uint8 CONTROL_TYPE_EFFORT=2
uint8 CONTROL_TYPE_VELOCITY=3
uint8 CONTROL_TYPE_POSITION_VELOCITY=4
uint8 CONTROL_TYPE_POSITION_VELOCITY_EFFORT=5
uint8 control_type                               # servomotor control type
