std_msgs/Header header

uint8 ENCODER_ROTARY=0
uint8 ENCODER_LINEAR=1
uint8 encoder_type

uint8 INCREMENTAL=0
uint8 ABSOLUTE=1             # one turn: after a complete revolution, the measurement values are repeated
uint8 ABSOLUTE_MULTI_TURN=2  # multi turn: apart from the angular position, the number of revolutions also counts
uint8 encoder_information

int32 max_speed              # Rotary encoder (rpm) // linear encoder (m)
int32 max_torque             # (N*m)

int32 resolution             # Rotary encoder in pulses per revolution (PPR) // linear encoder (m) // bits se usa unicamente para absolutos?
bool accuracy                # True: The pulses are identical/ False: not identical pulses
