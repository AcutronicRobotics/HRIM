# This message contains an uncompressed image
# If the camera is uncalibrated, the matrices D, K, R, P should be left zeroed out.  In particular,
# clients may assume that K[0] == 0.0 indicates an uncalibrated camera.
#  Time of image acquisition, camera coordinate frame ID
std_msgs/Header header             # Header timestamp should be acquisition time of image
                          # Header frame id should be optical frame of camera
                          # origin of frame should be optical center of camera
                          # +x should point to the right in the image
                          # +y should point down in the image
                          # +z should point into to plane of the image
# Calibration Parameters
# These are fixed during camera calibration.  Their values will be the same in
# all messages until the camera is recalibrated.  Note that self-calibrating
# systems may ”recalibrate” frequently.
# The internal parameters can be used to warp a raw (distorted) image to:
# 1.  An undistorted image (requires D and K)
# 2.  A rectified image (requires D, K, R)
# The projection matrix P projects 3D points into the rectified image.)
# The image dimensions with which the camera was calibrated.)

uint32 height             # image height, that is, number of rows in pixel
uint32 width              # image width, that is, number of columns in pixel

# The distortion model used.  Supported models are listed in sensor msgs/distortion_models.h.
#  For  most  cameras,  ”plumb_bob”  -  a  simple  model  of  radial  and  tangential  distortion  -  is sufficient.)
string distortion_model

# The distortion parameters, size depending on the distortion model.
# For ”plumb_bob”, the 5 parameters are:  (k1, k2, t1, t2, k3).
float64[] d

# Intrinsic camera matrix for the raw (distorted) images.
#     [fx 0 cx]
# K = [ 0 fy cy]
#     [ 0 0 1]
# Projects 3D points in the camera coordinate frame to 2D pixel coordinates using the
# focal lengths (fx, fy) and principal point (cx, cy).
float64[9] k              # 3x3 row-major matrix

# Projection/camera matrix
#     [fx’ 0 cx’ Tx]
# P = [ 0 fy’ cy’ Ty]
#     [ 0 0 1 0]

# By convention, this matrix specifies the intrinsic (camera) matrix of the processed (rectified)
# image.  That is, the left 3x3 portion is the normal camera intrinsic matrix for the rectified
# image.  It projects 3D points in the camera coordinate frame to 2D pixel coordinates using the
# focal lengths (fx’, fy’) and principal point (cx’, cy’) - these may differ from the values in K.
# For monocular cameras, Tx = Ty = 0.  Normally, monocular cameras will also have R = the
# identity and P[1:3,1:3] = K.
# For a stereo pair, the fourth column [Tx Ty 0]’ is related to the position of the optical center of
# the second camera in the first camera’s frame.  We assume Tz = 0 so both cameras are in the
# same stereo image plane.The first camera always has Tx = Ty = 0.  For the right (second)
# camera of a horizontal stereo pair, Ty = 0 and Tx = -fx’ * B, where B is the baseline between
# the cameras.
# Given a 3D point [X Y Z]’, the projection (x, y) of the point onto the rectified image is given by:
# [u v w]’ = P * [X Y Z 1]’
#       x = u / w
#       y = v / w
# This holds for both images of a stereo pair.
float64[12] p # 3x4 row-major matrix.
