# The main characteristics of the robotic arm

#Movement axes, relative to the origin:

uint16 ARM_TYPE_CARTESIAN=0
uint16 ARM_TYPE_CYLIDRICAL=1
uint16 ARM_TYPE_SPHERICAL=2
uint16 ARM_TYPE_SCARA=3
uint16 ARM_TYPE_ARTICULATED=4
uint16 ARM_TYPE_PARALLEL=5
uint16 arm_type                      # robotic arm type

uint16 dof                           # Numbers of degrees of freedom

uint16 max_payload                   # Maximum payload [kg]

bool x_axis                          # True: the robot can move in x-axis False: do not
bool y_axis                          # True: the robot can move in y-axis False: do not
bool z_axis                          # True: the robot can move in z-axis False: do not

int32 max_x_reach                    # x maximum position respect the origin [m]
int32 max_y_reach                    # y maximum position respect the origin [m]
int32 max_z_reach                    # z maximum position respect the origin [m]
