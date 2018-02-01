 #This message is to perform a precision analysis of the servo 
 #to be able to anticipate a possible deterioration or breakage
 
std_msgs/Header header                  # Header timestamp should be acquisition time of precision dates

float64 precision_min                   # The minimum precision that the application required in radians
                                        # (maximum error allowed)
float64 precision_first                 # The value of the first movement precision in radians
float64 precision_last                  # The value of the first movement precision in radians
