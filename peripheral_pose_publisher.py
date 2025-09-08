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
        "R": [0.583, -0.295, 0.012],
        "G": [0.506, -0.295, 0.012],
        "B": [0.444, -0.295, 0.012],
        "K": [0.382, -0.295, 0.012],
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

    # Positions for 4 color pads
    pad_positions = {
        "R": [0.56, 0.12, 0.04],
        "G": [0.49, 0.12, 0.04],
        "B": [0.49, 0.26, 0.04],
        "K": [0.56, 0.26, 0.04],
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
