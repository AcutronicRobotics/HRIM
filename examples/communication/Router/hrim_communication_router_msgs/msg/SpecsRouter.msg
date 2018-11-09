std_msgs/Header header

string router_ip                    # the ip to access inside the router
string user_name                    # user name to access inside the router
string password                     # password to access inside the router

uint32 max_devices                  # number of connected devices

float32 data_transfer               # data transfer velocity (Mbps)

#frecuency
uint8 FRECUENCY_24GHZ=0
uint8 FRECUENCY_5GHZ=1
uint8 FRECUENCY_24GHZ_5GHZ=2
uint8 frecuency                      # admited frecuency

#WiFi

uint8 WIFI_STANDARD_80211B=0
uint8 WIFI_STANDARD_80211A=1
uint8 WIFI_STANDARD_80211G=2
uint8 WIFI_STANDARD_80211N=3
uint8 WIFI_STANDARD_80211AC=4
uint8 wifi_standard                  # WiFi standard: range, frequency band...

uint8 WIFI_SECURITY_WEP=0
uint8 WIFI_SECURITY_WPA=1
uint8 WIFI_SECURITY_WPA2=2
uint8 security                       # Security of the network
