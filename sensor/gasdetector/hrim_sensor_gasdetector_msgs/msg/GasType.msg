string detect_gas_name            # The gases that the sensor can detect
string gas_detection_technique    # Type of the sensor:  catalytic, ECC,
                                  # Point IR, Open Path,
                                  # PID, Semiconductor, Paper tape.
uint8 HROS_GASSENSOR_CATALYTIC=0
uint8 HROS_GASSENSOR_ECC=1
uint8 HROS_GASSENSOR_POINTIR=2
uint8 HROS_GASSENSOR_OPENPATH=3
uint8 HROS_GASSENSOR_PID=4
uint8 HROS_GASSENSOR_SEMICONDUCTOR=5
uint8 HROS_GASSENSOR_PAPERTAPE=6
uint8 HROS_GASSENSOR_OTHER=7

uint8 detection_range              # maximum value (fps)
