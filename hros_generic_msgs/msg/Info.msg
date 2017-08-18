std_msgs/Header header          # communication master state at this time

string vendor                   # vendor name
string id                       # vendor ID
string part_number              # part number
string serial_number            # serial number
string hros_version             # HROS version - API.addfunctionality.bugfixes
uint16 COMM_TYPE_ETHERNET_10=0
uint16 COMM_TYPE_ETHERNET_100=1
uint16 COMM_TYPE_ETHERNET_1000=2
uint16 COMM_TYPE_USB=3
uint16 comm_type                # communication type

uint8 HROS_COMM=0
uint8 HROS_SENSOR=1
uint8 HROS_ACTUATOR=2
uint8 HROS_COGNITION=3
uint8 hros_type                 # HROS node type
