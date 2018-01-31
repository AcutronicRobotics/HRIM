# This message contains a distance

std_msgs/Header header      # timestamp in the header is the time the ranger
                            # returned the distance reading

float32 range_min           # minimum range value in metres
float32 range_max           # maximum range value in metres

float32 time_measurements   # time between measurements in seconds

float32 distance            # range data in metres
