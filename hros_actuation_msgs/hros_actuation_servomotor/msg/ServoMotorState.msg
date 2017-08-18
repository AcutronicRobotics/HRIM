std_msgs/Header header        # Motor state is at this time

float64 goal                  # commanded position (rad)
float64 position              # current position encoder (rad)
float64 error                 # difference between current and goal positions (rad)
float64 speed                 # current speed (rad/s)
float64 load                  # current weight can move
bool moving                   # whether the motor is currently in motion

#Fault states which could cause the driver to be immobilized.

uint8 FAULT_NONE=0
uint8 FAULT_CURRENT=1
uint8 FAULT_TEMPERATURE=2
uint8 FAULT_BUS_VOLTAGE=3
uint8 FAULT_BRIDGE_DRIVER=4
uint8 fault                   # fault cause
