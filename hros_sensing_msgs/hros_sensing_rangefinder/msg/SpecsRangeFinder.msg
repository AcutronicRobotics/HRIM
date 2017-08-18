# This message define the most important characteristic of the range finder

std_msgs/Header header

uint8 ULTRASOUND=0
uint8 INFRARED=1
uint8 radiation_type      # the type of radiation used by the sensor

float32 field_of_view     # the size of the arc that the distance reading is valid for [rad]
                          # 0 angle corresponds to the x-axis of the sensor.

float32 min_range         # minimum range value [m]
float32 max_range         # maximum range value [m]
float32 max_sample_rate   # maximum sample rate [Hz]
