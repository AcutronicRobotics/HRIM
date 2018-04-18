std_msgs/Header header

uint8 MOTOR_DC_BRUSHED=0
uint8 MOTOR_DC_BRUSHLESS=1
uint8 MOTOR_DC_GEAREDDOWN=2
uint8 motor_type                  # motor type

bool encoder                      # True: the motor has movement control device
                                  # False: it does not

int32 rated_power                 # (W)
int32 rated_voltage               # (V)
int32 rated_current               # (A)
int32 rated_torque                # (N*m)
int32 rated_speed                 # (rpm)

int32 noload_current              # current without load (A)
int32 noload_speed                # speed without load (rpm)

int32 stall_torque                # torqye without speed (N*m)
