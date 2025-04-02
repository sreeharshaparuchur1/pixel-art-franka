#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray


def to_homogeneous_matrix(x, y, z):

    T = np.identity(4)
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z

    return T


def create_stamp_pose_message():
    # Dummy positions for 4 color stamps
    stamp_positions = {
        "R": [0.579, -0.295, 0.012],
        "G": [0.382, -0.290, 0.012],
        "B": [0.444, -0.290, 0.012],
        "K": [0.506, -0.290, 0.012],
    }

    float_array = []

    for color in ["R", "G", "B", "K"]:
        x, y, z = stamp_positions[color]
        T = to_homogeneous_matrix(x, y, z)
        float_array.extend(T.flatten())  # Add 16 values

    msg = Float32MultiArray()
    msg.data = float_array
    return msg


def create_pad_pose_message():

    # Dummy positions for 4 color pads
    pad_positions = {
        "R": [0.5, 0.2, 0.03],
        "G": [0.5, 0.35, 0.03],
        "B": [0.6, 0.35, 0.03],
        "K": [0.6, 0.2, 0.03],
    }

    float_array = []

    for color in ["R", "G", "B", "K"]:
        x, y, z = pad_positions[color]
        T = to_homogeneous_matrix(x, y, z)
        float_array.extend(T.flatten())  # Add 16 values

    msg = Float32MultiArray()
    msg.data = float_array
    return msg


if __name__ == "__main__":
    rospy.init_node("pose_publisher")
    pad_pub = rospy.Publisher("/pad_pose_array", Float32MultiArray, queue_size=10)
    stamp_pub = rospy.Publisher("/stamp_pose_array", Float32MultiArray, queue_size=10)

    rate = rospy.Rate(1)
    rospy.loginfo(
        "Publishing dummy stamp and pad poses to /stamp_pose_array and /pad_pose_array..."
    )

    while not rospy.is_shutdown():
        pad_msg = create_pad_pose_message()
        stamp_msg = create_stamp_pose_message()

        pad_pub.publish(pad_msg)
        stamp_pub.publish(stamp_msg)

        rate.sleep()


# Initial
# joints:
# [-0.3319039   0.44236546 -0.01032947 -2.43292841  0.09381642  2.82011236
#   0.3366749 ]

# Sreeharsha reads the first pixel as:
# [-0.31114144  0.46055729 -0.03970958 -2.45778251  0.03399489  2.93705922, 0.35787311]\

# FInal
# joints:
# [ 0.66987715  0.58243003 -0.2269621  -2.20325092  0.15133892  2.8338912
#   1.10996746]

# Sreeharsha reads the stamp joint angles as:
# [ 0.21074577  0.59300807  0.14203533 -2.12706332 -0.1295534   2.67750665  1.26011138]
