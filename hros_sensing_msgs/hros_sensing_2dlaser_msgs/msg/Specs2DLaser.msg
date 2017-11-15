std_msgs/Header header          # timestamp in the header is the acquisition time of
                                # the first ray in the scan.

float32 field_of_view           # aperture/scaning angle in radians
float32 angular_accuracy        # angular resolution in radians

float32 min_angle_increment     # minimum angle between measurements in radians
float32 min_scan_time           # minimum time between measurements in seconds

float32 range_min               # operating minimum range in meters
float32 range_max               # operating maximum range in meters
