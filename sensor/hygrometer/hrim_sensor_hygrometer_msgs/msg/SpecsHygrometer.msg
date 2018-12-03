# This message define the most important characteristic of the hygrometer

std_msgs/Header header

float32 resolution	# Minimum change the hygrometer detects (%) I.E. 0.01

float32 max_calibrated_rh	# Maximum relative humidity where the detection accuracy is at it's best (rh)
float32 min_calibrated_rh	# Minimum relative humidity where the detection accuracy is at it's best (rh)

float32 max_calibrated_temp	# Maximum temperature where the detection accuracy is at it's best (°C)
float32 min_calibrated_temp	# Minimum temperature where the detection accuracy is at it's best (°C)

float32 accuracy_calibrated	# Average error margin while inside the calibrated ranges (+-rh)
float32 accuracy_other	# Average error margin while outside the calibrated ranges (+-rh)

float32 max_rh 100	# Maximum operating relative humidity, 100 if unspecified (rh)
float32 min_rh 0	# Minimum operating relative humidity, 0 if unspecified (rh)

float32 max_temp	# Maximum operating temperature (°C)
float32 min_temp	# Minimum operating temperature (°C)

float32 max_measure_time  # Maximum amount of time needed for a measurement (s)
float32 min_measure_time  # Minimum amount of time needed for a measurement (s)

uint8 CAPACITIVE=0
uint8 RESISTIVE=1
uint8 THERMAL=2
uint8 GRAVIMETRIC=3
uint8 device_type # The way the hygrometer detects relative humidity
