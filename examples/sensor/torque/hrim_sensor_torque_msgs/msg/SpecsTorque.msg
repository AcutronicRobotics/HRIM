std_msgs/Header header

uint16 max_torque           # Maximum measurable torque (Nm)

float32 resolution          # Minimum detectable difference (Nm)

uint8 STATIC=0
uint8 ROTARY=1
uint8 NONCONTACTING=2
uint8 device_type           # The way the sensor detects torque

uint8 CLOCKWISE=0
uint8 COUNTERCLOCKWISE=1
uint8 BIDIRECTIONAL=2
uint8 sensor_direction      # The rotatory direction the torque sensor measures

uint32 max_rpm 0            # The maximum rpm the sensor withstands, if the sensor type is static leave default (RPM)

uint16 safe_overload        # Maximum force the sensor withstands safely, in reference to the max_torque value (%)
uint16 max_overload         # Maximum force the sensor withstands before total breakdown, in reference to the max_torque value (%)

uint16 max_rate             # Maximum amount of measurements per second (Hz)

float32 min_ideal_temp	    # Minimum temperature where the error margin is at it's lowest (°C)
float32 max_ideal_temp	    # Maximum temperature where the error margin is at it's lowest (°C)

float32 min_operating_temp  # Minimum operating temperature (°C)
float32 max_operating_temp	# Maximum operating temperature (°C)

float32 min_error_margin	  # Average error margin while inside the ideal temperature ranges (±%)
float32 max_error_margin	  # Average error margin while outside the ideal temperature ranges (±%)
