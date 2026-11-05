# Published once by the team on /parc/tag_visit at each tag, while the robot is
# stopped in the visit pose (0.5 m from the tag, facing it within +-12 degrees).
# After publishing, the robot must hold the pose for at least 3 seconds.
# QoS: reliable, depth 10.

# header.stamp: the moment of the visit. header.frame_id: the camera frame used.
std_msgs/Header header

# AprilTag 36h11 ID of the tag being visited (1, 4, 7 or 9).
uint8 tag_id

# JPEG image from a robot camera in which the tag is clearly visible.
sensor_msgs/CompressedImage image
