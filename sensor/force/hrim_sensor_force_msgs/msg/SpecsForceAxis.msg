# Specs for a single axis of a force sensor

uint8 FORCE_AXIS_X=0
uint8 FORCE_AXIS_Y=1
uint8 FORCE_AXIS_Z=2
uint8 axis # The axis the specs describe

uint16 max_force # Maximum measurable force (N)

float32 resolution # Minimum detectable difference (N)

bool bidirectional  # If the axis also measures negative values or not (true,false)

uint16 safe_overload # Maximum force the sensor withstands safely on this axis, in reference to the max_force value (%)
uint16 max_overload  # Maximum force the sensor withstands before total breakdown on this axis, in reference to the max_force value (%)

float32 min_error_margin	# Average error margin while inside the ideal ranges of this axis (±%)
float32 max_error_margin	# Average error margin while outside the ideal ranges of this axis (±%)
