# Specs for a single torque axis of a force & torque sensor

uint8 MX=0
uint8 MY=1
uint8 MZ=2
uint8 axis # The torque axis the specs describe

uint16 max_torque # Maximum measurable torque (Nm)

float32 resolution # Minimum detectable difference (Nm)

uint16 safe_overload # Maximum force the sensor withstands safely on this axis, in reference to the max_torque value (%)
uint16 max_overload  # Maximum force the sensor withstands before total breakdown on this axis, in reference to the max_torque value (%)

float32 min_error_margin	# Average error margin while inside the ideal ranges of this axis (±%)
float32 max_error_margin	# Average error margin while outside the ideal ranges of this axis (±%)
