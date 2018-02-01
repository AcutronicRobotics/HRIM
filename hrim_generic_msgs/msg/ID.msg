std_msgs/Header header

uint8 HRIM_COMM=0
uint8 HRIM_SENSOR=1
uint8 HRIM_ACTUATOR=2
uint8 HRIM_COGNITION=3
uint8 HRIM_UI=4
uint8 HRIM_POWER=5
uint8 HRIM_COMPOSITE=6
unit8 device_kind_id     # device classification

string device_name       # sub-type, for example, camera
uint32 vendor_id         # identifier to identify the vendor
unit32 product_id        # part-number
unit32 instance_id       # unique number to identify the module

string hrim_version      # used hrim version
string hros_version      # used h-ros version
