std_msgs/Header header

uint8 CHARGE_COMPLETED=0
uint8 BATTERY_LOW=1
uint8 BATTERY_CRITICAL=2
uint8 battery_status                          # status of the battery

float32 power_consumption                     # can only be calculated if not charging

float32 relative_remaining_capacity           # percent of maximum capacity
float32 remaining_capacity                    # (Ah)

bool charging                                 # flag if robot is connected to external power or NOT
float32 time_remaining_charging               # estimated time to empty or fully charged (h)
