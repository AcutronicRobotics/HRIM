std_msgs/Header header

string name			# hros name

# CPU

float32 system_cpu		# Total CPU utilization in %
float32 core_temperature	# CPU temperature in C

# RAM
uint16 system_ram_total		# Total RAM in MB (megabyte)
uint16 system_ram_used		# Used RAM in MB (megabyte)
uint16 system_ram_free		# Free RAM in MB (megabyte)

# Disk I/O
float32 system_io_in		# Disk I/O IN in kilobytes/s
float32 system_io_out		# Disk I/O OUT in kilobytes/s

# IPv4 Bandwidth
string[] system_ipv4_ip		# IP address (xxx.xxx.xxx.xxx/zz) *
float32 system_ipv4_received	# IPv4 bandwidth received in kilobits/s
float32 system_ipv4_sent	# IPv4 bandwidth sent in kilobits/s

# IPv4
uint16 ipv4_tcpsock		# Number of TCP active connections
float32 ipv4_tcppackets_received# Number of TCP received packets
float32 ipv4_tcppackets_sent	# Number of TCP sent packets
float32 ipv4_tcp_errors		# Number of TCP error packages
float32 ipv4_udppackets_received# Number of UDP packets received
float32 ipv4_udppackets_sent	# Number of UDP packets sent
float32 ipv4_udp_errors		# Number of UDP error packages
