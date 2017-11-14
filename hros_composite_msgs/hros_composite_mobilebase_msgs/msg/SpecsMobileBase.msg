std_msgs/Header header            


uint16 MOBILEBASE_TYPE_WHEEL=0
uint16 MOBILEBASE_TYPE_TRACK=1
uint16 MOBILEBASE_TYPE_LEG=2
uint16 base_type                      # mobile base robot type

# Weeled robot characteristics

int32 max_velocity                       # maximum vehicle velocity [m/s]
uint16 number_wheel                      # how many weels the base has
uint16 number_motor                      # how many motors the base has

uint16 WHEELED_ROBOT_TYPE_DIFFERENTIAL=0
uint16 WHEELED_ROBOT_TYPE_SKID=1
uint16 WHEELED_ROBOT_TYPE_CAR=2
uint16 WHEELED_ROBOT_TYPE_OMNI=3
uint16 WHEELED_ROBOT_MECANUM=4
uint16 base_type                        # mobile base robot type


# Tracked robot characteristics

int32 max_velocity                      # maximum vehicle velocity [m/s]

bool flipper_stabilizer                 # True: it contains this characteristic/False: do not

# Legged robot characteristics

int32 max_velocity                      # maximum vehicle velocity [m/s]
uint16 number_legs                      # how many legs the base has
uint16 number_dof                       # how many degrees of freedom
