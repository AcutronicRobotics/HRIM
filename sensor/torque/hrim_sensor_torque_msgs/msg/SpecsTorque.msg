std_msgs/Header header

uint16 min_torque # Minimum measurable torque (Nm)
uint16 max_torque # Maximum measurable torque (Nm)

uint8 STATIC=0
uint8 ROTARY=1
uint8 NONCONTACTING=2
uint8 device_type # The way the sensor detects force

uint8 CLOCKWISE=0
uint8 COUNTERCLOCKWISE=1
uint8 BIDIRECTIONAL=2
uint8 force_direction # The rotatory direction the torque sensor measures

uint32 max_rpm 0  # The maximum rpm the sensor withstands, if the sensor type is static leave default (RPM)

uint16 safe_overload # Maximum force the sensor withstands safely, in reference to the max_rated value (%)
uint16 max_overload  # Maximum force the sensor withstands before total breakdown, in reference to the max_rated value (%)

uint8 axis # Number of axis the torque sensor detects (1-3)

uint16 max_rate # Maximum amount of measurements per second (Hz)

float32 min_calibrated_temp	# Minimum temperature where the detection accuracy is at it's best (°C)
float32 max_calibrated_temp	# Maximum temperature where the detection accuracy is at it's best (°C)

float32 min_operating_temp # Minimum operating temperature (°C)
float32 max_operating_temp	# Maximum operating temperature (°C)

float32 accuracy_calibrated	# Average error margin while inside the calibrated ranges (±%)
float32 accuracy_other	# Average error margin while outside the calibrated ranges (±%)

bool force_sensor # If the sensor has force measuring capabilities or not (True/False)
