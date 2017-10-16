# This message contains the gases that the sensor can detect
std_msgs/Header header        # Header timestamp should be
                              # acquisition of the gas
                              # measurements
hros_sensing_gasdetector_msgs/GasMeasurement[] measurements # the measurement of each sensor
