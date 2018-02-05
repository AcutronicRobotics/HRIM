# This message define the most important characteristic of the camera
#
# It defines the possibilities the camera can provide
#
# The resolution should be widthxheight in pixels, and it has to reflect a list of
# possibilities in which the camera can work.
#
# The frame rate should be the maximum value that the camera can support in
# frames per seconds
#
# The image format should be a list of the possibilities that the camera provide
#
# You have to reflect the extras that tha camera has, for the moment PTZ and microphone
# if the camera has PTZ characteristic you have to show the vision range in degrees
# of Pan and Tild, if not put 0 value

std_msgs/Header header

uint32 instance_id

string resolution           # Example: 752x480, 1280x1024, ...  (px)
uint32 frame_rate           # maximum value (fps)
string image_formats        # Example: mono8, rgb1, bayergrbg8, ...

bool ptz                    # true:the camera has this extra; false:  do not.
float32 pan_vision_range     # Example: 360
float32 tilt_vision_range    # Example: 135
bool microphone             # true:the camera has this extra; false:  do not
