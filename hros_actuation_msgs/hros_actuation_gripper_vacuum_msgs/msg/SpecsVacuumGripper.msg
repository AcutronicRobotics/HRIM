std_msgs/Header header           # vacuum gripper state at this time

float32 cycle_time               # Needed time for one cycle [s]

float32 max_force                # Taking into account the maximum vacuum (100%) and with non-porous material [N]
float32 max_payload              # Maximum recommended payload [kg]
