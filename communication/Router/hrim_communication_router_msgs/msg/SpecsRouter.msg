std_msgs/Header header

uint32 max_devices                  # number of connected devices

float32 data_transfer               # data transfer velocity (Mbps)

#frecuency
uint8 FRECUENCY_2.4Ghz=0
uint8 FRECUENCY_5Ghz=1
uint8 FRECUENCY_2.4Ghz_5Ghz=2
uint8 frecuency                      # admited frecuency

#WiFi

uint8 WiFi_standard_802.11b=0
uint8 WiFi_standard_802.11a=1
uint8 WiFi_standard_802.11g=2
uint8 WiFi_standard_802.11n=3
uint8 WiFi_standard_802.11ac=4
uint8 WiFi_standard                  # WiFi standard: range, frequency band...

uint8 WiFi_security_WEP=0
uint8 WiFi_security_WPA=1
uint8 WiFi_security_WPA2=2
uint8 security                       # Security of the network
