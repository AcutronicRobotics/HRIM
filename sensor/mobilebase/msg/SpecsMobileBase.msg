std_msgs/Header header

uint8 MOBILEBASE_TYPE_WHEEL=0
uint8 MOBILEBASE_TYPE_TRACK=1
uint8 MOBILEBASE_TYPE_LEG=2
uint8 mobilebase_type                    # mobile base robot type

# Wheeled robot characteristics

float32 max_wheeled_velocity             # maximum vehicle velocity [m/s]
uint16 number_wheel                      # how many weels the base has
uint16 number_motor                      # how many motors the base has

uint8 WHEELED_ROBOT_TYPE_DIFFERENTIAL=0
uint8 WHEELED_ROBOT_TYPE_SKID=1
uint8 WHEELED_ROBOT_TYPE_CAR=2
uint8 WHEELED_ROBOT_TYPE_OMNI=3
uint8 WHEELED_ROBOT_MECANUM=4
uint8 wheeled_type                          # mobile base robot type

# Tracked robot characteristics

float32 max_tracked_velocity             # maximum vehicle velocity [m/s]
bool flipper_stabilizer                  # True: it contains this characteristic/False: do not

# Legged robot characteristics

float32 max_legged_velocity              # maximum vehicle velocity [m/s]
uint16 number_legs                       # how many legs the base has
uint16 number_dof                        # how many degrees of freedom
