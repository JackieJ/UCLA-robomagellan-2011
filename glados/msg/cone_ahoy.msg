# This represents a cone sighting event. The yaw angle is the extrinsic rotation
# (in radians), about the z-axis of the coordinate frame given by header.frame_id,
# that places the cone on the rotated x-axis. Positive to the left, negative to
# the right, zero for dead ahead. A distance of zero is an unknown distance.
Header header
float64 distance
float64 yaw_angle
