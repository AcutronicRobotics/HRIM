# Stereo camera

std_msgs/Header header

string resolution           # Example: 752x480, 1280x1024, ...  (px)
uint32 frame_rate           # maximum value (fps)
string image_formats        # Example: mono8, rgb1, bayergrbg8, ...

float32 baseline            # distances between the two camera lenses
float32 horizontal_fov      # horizontal field of view in radians
float32 vertical_fov        # vertical field of view in radians
string depth_range          # depth range in meters
