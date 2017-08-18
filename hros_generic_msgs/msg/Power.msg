std_msgs/Header header # component state at this time

# Measurements of components that are feed by PoE
float32 poe_voltage                 # Voltage measurement of PoE in Volts,
float32 poe_current                 # Current measurement, consumed current in Amperes
float32 poe_power                   # Power measurement, consumed power in Watts (P=V*I in DC)

# Supply:  Measurements of components that are feed by an external power supply
float32 supply_voltage              # Injected Voltage
float32 supply_current_consumption  # The current used for the device by the injected
                                    # power supply
float32 supply_current_surplus      # The current injecting into H-ROS,
                                    # Available for feeding to other components
float32 supply_power_consumption    # The power used for the device by the
                                    # injected power supply (P=V*I in DC)
float32 supply_power_surplus        # The power injecting into H-ROS, Available
                                    # for feeding to other components (P=V*I in DC)
