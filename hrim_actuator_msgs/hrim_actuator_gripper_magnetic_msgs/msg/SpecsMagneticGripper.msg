# The main chacarteristics of the magnetic gripper

std_msgs/Header header                   # magnetic gripper state at this time

float32 magnetic_force                   # The force of the magnet [N]
float32 max_payload                      # Maximum payload [kg]


bool vacuum                              # True: the magnetic gripper include vacuum
                                         # False: do not
float32 max_vacuum_force                 # The force of the vacuum [N]
                                         # If the magnetic gripper do not include vacuum = 0
                                         # If it includes max_total_gripper_force =  max_magnetic_force + max_vacuum_force
