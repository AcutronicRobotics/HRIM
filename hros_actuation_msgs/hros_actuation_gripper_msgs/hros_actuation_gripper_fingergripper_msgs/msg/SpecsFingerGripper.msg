# The main chacarteristics of the finger gripper

std_msgs/Header header                # Finger gripper state at this time

float32 min_force                        # Minimum gripping force [N]
float32 max_force                        # Maximum gripping force [N]

float32 max_payload                      # Maximum recommended payload [kg]

float32 min_speed                        # Minimum closing speed [mm/s]
float32 max_speed                        # Maximum closing speed [mm/s]
                                      # If max=min not possible to change the speed

float32 max_acceleration                 # Maximum acceleration [mm/s]
                                      # If 0 = does not have this ability

# There are two types of finger grippers taking into acoount the path of the fingers: linear or angular.
# One of them has to be =0 implying that it is not the technique it uses:

float32 max_length                       # Maximum permitted finger length [mm]
float32 max_angle                        # Maximum permitted finger angle [rad]

float32 repeatability                    # Parallel grip repeatability [s]

float32 max_vacuum_force                 # The force of the vacuum [N]
                                      # If the magnetic gripper do not include vacuum = 0
                                      # If it includes max_total_gripper_force =  max_magnetic_force + max_vacuum_force
