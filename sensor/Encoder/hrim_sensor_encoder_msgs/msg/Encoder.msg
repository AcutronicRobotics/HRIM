std_msgs/Header header

bool moving                         # whether the motor is currently in motion

# Explained in SpecsEncoder.msg
uint8 encoder
uint8 encoder_type
uint8 encoder_output

int32 goal_position                 # commanded position (rad)
int32 current_position              # current position (rad)
int32 error_position                # difference between current and goal positions (rad)

int32 goal_speed                    # commanded position (rpm)
int32 current_speed                 # current position (rpm)
int32 error_speed                   # difference between current and goal speed (rpm)

int32 goal_acceleration             # commanded position (rpm2)
int32 current_acceleration          # current position (rpm2)
int32 error_acceleration            # difference between current and goal acceleration (rpm2)
