std_msgs/Header header            # timestamp in the header is the acquisition time of
                                  # the first ray in the scan.
                         
float32 angle_start               # start angle of the scan in radians
float32 angle_end                 # end angle of the scan in radians

float32 angle_increment           # angular distance between measurements in radians
float32 time_increment            # time between scans in seconds - if your scanner
                                  # is moving, this will be used in interpolating position
                                  # of 3d points
float32 time_scan                 # time between scans in seconds

float32 range_min                 # actual minimum range value in metres
float32 range_max                 # actual maximum range value in metres

float32[] ranges                  # range data [m] (Note: values < range_min or > range_max should be discarded)
float32[] intensities             # intensity data [device-specific units].  If your
                                  # device does not provide intensities, please leave
                                  # the array empty.
