#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray

def to_homogeneous_matrix(x, y, z):

    T = np.eye(4)
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z

    return T

def create_pad_pose_message():

    # Dummy positions for 4 color pads
    pad_positions = {
        'R': [0.4, -0.2, 0.03],
        'G': [0.6, -0.25, 0.03],
        'B': [0.5, -0.1, 0.03],
        'K': [0.45, -0.3, 0.03]
    }

    float_array = []

    for color in ['R', 'G', 'B', 'K']:
        x, y, z = pad_positions[color]
        T = to_homogeneous_matrix(x, y, z)
        float_array.extend(T.flatten())  # Add 16 values

    msg = Float32MultiArray()
    msg.data = float_array
    return msg    

if __name__ == '__main__':
    rospy.init_node('dummy_pad_pose_publisher')
    pub = rospy.Publisher('/pad_pose_array', Float32MultiArray, queue_size=10)

    rate = rospy.Rate(1)
    rospy.loginfo("Publishing dummy pad poses to /pad_pose_array...")

    while not rospy.is_shutdown():
        msg = create_pad_pose_message()
        pub.publish(msg)
        rate.sleep()
