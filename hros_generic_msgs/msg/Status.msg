std_msgs/Header header

string name			# hros name

# CPU

float32 system_cpu		# Total CPU utilization in %
float32 core_temperature	# CPU temperature in C

# RAM
uint32 system_ram_total		# Total RAM in MB (megabyte)
uint32 system_ram_used		# Used RAM in MB (megabyte)
uint32 system_ram_free		# Free RAM in MB (megabyte)

# Disk I/O
float32 system_io_in		# Disk I/O IN in kilobytes/s
float32 system_io_out		# Disk I/O OUT in kilobytes/s

# IPv4 Bandwidth
#string[] system_ipv4_ip		# IP address (xxx.xxx.xxx.xxx/zz) *
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

float32 cpu_interrupts
float32 cpu_context_switches

uint32 softnet_processed
uint32 softnet_dropped
uint32 softnet_squeezed
uint32 softnet_received_rps
uint32 softnet_flow_limit_count
uint64 softirqs_hi
uint64 softirqs_timer
uint64 softirqs_net_tx
uint64 softirqs_net_rx
uint64 softirqs_block
uint64 softirqs_irq_poll
uint64 softirqs_tasklet
uint64 softirqs_sched
uint64 softirqs_hrtimer
uint64 softirqs_rcu

float32 load1
float32 load5
float32 loda15
