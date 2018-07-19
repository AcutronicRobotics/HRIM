std_msgs/Header header

uint8 BATTERY_NICD=1
uint8 BATTERY_NIMH=2
uint8 BATTERY_LIFEPO=3
uint8 BATTERY_LIPO=4
uint8 BATTERY_LIION=5
uint8 battery_type

float32 voltage                # (V)
float32 current                # (A)
int32 capacity                 # (mAh)

int32[] working_temp           # working temperature range (celsius)

# LiPo
uint8 cells                    # number of cells
float32 cell_voltage           # voltage per cell (V)
float32 charge_voltage         # charge cut-off voltage (V)
float32 charge_current         # charge cut-off current (A)
float32 discharge_voltage      # discharge cut-off voltage (V)
int32 impedance                # (mΩ)
