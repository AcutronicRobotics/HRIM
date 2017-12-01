# This message define the most important characteristic of the Depth sensor camera

# It defines the possibilities the camera can provide

# The resolution should be widthxheight in pixels, and it has to reflect a list of
# possibilities in which the camera can work.

# The frame rate should be the maximum value that the camera can support in
# frames per seconds

# The image format should be a list of the possibilities that the camera provide

std_msgs/Header header

string resolution         # Example:  752x480, 1280x1024, ...  (width x height) (px)

string fov                  # Field of view (horizontalxverticalxdiagonal) (degrees)
uint16 max_frame_rate       # maximum value (fps)
float32 accuracy             # maximum value (m)
uint16[] operation_range      # working distance (min-max)(m)
string image_formats      # Example:  mono8, rgb1, bayergrbg8, ...
float32 voltage              # (min-max) (V)
float32 max_power_consumption# Power consumption working full (W)
bool rgb                    # true:image in RGB (color); false:  image in grayscale.
bool microphone             # true:the camera has this extra; false:  do not.
bool lighting               # true:the camera has LEDs; false:  do no
