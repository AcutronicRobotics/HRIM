std_msgs/Header header

uint8 BATTERY_NiCd=1
uint8 BATTERY_NiMH=2
uint8 BATTERY_LiFePo=3
uint8 BATTERY_LiPo=4
uint8 BATTERY_LiIon=5
uint8 battery_type

float16 voltage                # (V)
float16 current                # (A)
int16 capacity                 # (mAh)

int16 working_temp[]           # working temperature range (celsius)

# LiPo
uint8 cells                    # number of cells
float16 cell_voltage           # voltage per cell (V)
float16 charge_voltage         # charge cut-off voltage (V)
float16 charge_current         # charge cut-off current (A)
float16 discharge_voltage      # discharge cut-off voltage (V)
float16 charge_current         # discharge cut-off current (A)
int16 impedance                # (mΩ)
