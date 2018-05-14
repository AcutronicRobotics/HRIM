std_msgs/Header header

uint8 CHARGE_COMPLETED=0
uint8 BATTERY_LOW=1
uint8 BATTERY_CRITICAL=2
uint8 battery status                          # status of the battery

float16 power_consumption                     # can only be calculated if not charging

float16 relative_remaining_capacity           # percent of maximum capacity
float16 remaining_capacity                    # (Ah)

bool charging                                 # flag if robot is connected to external power or NOT
float16 time_remaining_charging               # estimated time to empty or fully charged (h)
