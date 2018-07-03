# The main characteristics of the robotic arm

std_msgs/Header header

uint8 ARM_TYPE_CARTESIAN=0
uint8 ARM_TYPE_CYLIDRICAL=1
uint8 ARM_TYPE_SPHERICAL=2
uint8 ARM_TYPE_SCARA=3
uint8 ARM_TYPE_ARTICULATED=4
uint8 ARM_TYPE_PARALLEL=5
uint8 arm_type                        # robotic arm type

uint32 dof                            # Numbers of degrees of freedom

float32 max_payload                   # Maximum payload [kg]

#Movement axes, relative to the origin:

bool x_axis                            # True: the robot can move in x-axis False: do not
bool y_axis                            # True: the robot can move in y-axis False: do not
bool z_axis                            # True: the robot can move in z-axis False: do not

float32 max_x_reach                    # x maximum position respect the origin [m]
float32 max_y_reach                    # y maximum position respect the origin [m]
float32 max_z_reach                    # z maximum position respect the origin [m]
