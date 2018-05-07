std_msgs/Header header

bool internet_connected         # True:connected to Internet / False: Disconnected

uint8 INTERNET_4G=0
uint8 INTERNET_ETHERNET=1
uint8 working_mode

float32 amount_data            # If the connection is by 4G, data left (Gb)

# working velocity
float32 actual_down              # data download velocity (Mbps)
float32 actual_up                # data upload velocity (Mbps)      
