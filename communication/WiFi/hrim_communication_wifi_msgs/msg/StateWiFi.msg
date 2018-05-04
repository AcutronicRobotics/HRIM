std_msgs/Header header

bool wifi_connected         # True:connected to WiFi / False: Disconnected

uint8 FRECUENCY_2.4Ghz=0
uint8 FRECUENCY_5Ghz=1
uint8 frecuency                      # frequency to which you are connected

# working velocity
float32 actual_down              # data download velocity (Mbps)
float32 actual_up                # data upload velocity (Mbps)   
