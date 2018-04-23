std_msgs/Header header          # timestamp in the header is the acquisition time of
                                # the first ray in the scan.

float32 horizontal_fov          # horizontal field of view in radians
float32 vertical_fov            # vertical field of view in radians
float32 linear_accuracy         # linear resolution in meters
float32 angular_accuracy        # angular resolution in radians

float32 min_angle_increment     # minimum angle between measurements in radians
float32 min_scan_time           # minimum time between measurements in seconds

float32 distance_min            # operating minimum distance in meters
float32 distance_max            # operating maximum distance in meters
float32 points_max              # maximum points per second

bool intensity                 
