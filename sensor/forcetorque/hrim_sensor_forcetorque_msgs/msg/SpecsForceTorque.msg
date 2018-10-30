# Specs for a force & torque sensor

std_msgs/Header header

hrim_sensor_forcetorque_msgs/SpecsForceAxis[<=3] force_axis_specs # Specs related to each of the measured force axes
hrim_sensor_forcetorque_msgs/SpecsTorqueAxis[<=3] torque_axis_specs # Specs related to each of the measured torque axes

uint16 max_rate # Maximum amount of measurements per second (Hz)

float32 min_ideal_temp	# Minimum temperature where the error margin is at it's lowest (°C)
float32 max_ideal_temp	# Maximum temperature where the error margin is at it's lowest (°C)

float32 min_operating_temp  # Minimum operating temperature (°C)
float32 max_operating_temp	# Maximum operating temperature (°C)
