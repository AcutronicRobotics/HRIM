std_msgs/Header header

hrim_sensor_gasdetector_msgs/GasType[] particular_specs  # specs of each gas that the sensor
                                                        # detects
uint32 response_time                             # The seconds that the sensor needs
                                              # to refresh
bool display
                                              # true:  the gas detector has this extra;
                                              # false:  do not.
bool alarm
                                              # true:  the gas detector has this extra;
                                              # false:  do not.
bool leds
                                              # true:  the gas detector has this extra;
                                              # false:  do not.
bool wifi
                                              # true:  the gas detector has this extra;
                                              # false:  do not.
bool gps
                                              # true:  the gas detector has this extra;
                                              # false:  do no
