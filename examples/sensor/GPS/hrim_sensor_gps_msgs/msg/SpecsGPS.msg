std_msgs/Header header

uint8 channels                  # ammount of channels
string frecuency                # L1 or L2

int32 min_update_rate           # minimum update rate in Hertz
int32 max_update_rate           # maximum update rate in Hertz

float32 max_altitude            # operational limit, maximum altitude in meters
int32   max_velocity            # operational limit, maximum velocity in meters per second

float32 gps_accuracy            # horizontal position accurary in meters

string[] sbas_system            # list of supported Satellite Based Augmentation System
float32 sbas_accuracy           # accuracy with SBAS in meters
