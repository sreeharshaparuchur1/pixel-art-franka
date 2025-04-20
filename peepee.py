#!/usr/bin/env python3

import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
from networkx.algorithms.approximation import christofides
import rospy
from std_msgs.msg import String, Float32MultiArray
from frankapy import FrankaArm
from autolab_core import RigidTransform
import time
import pdb


if __name__ == "__main__":
    fa = FrankaArm()
    fa.reset_joints()

    des_pose = RigidTransform(rotation=np.array([
        [1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 0.0, -1.0]]),
        translation=np.array([0.382, -0.295, 0.03]),
        from_frame='franka_tool', to_frame='world')
    
    fa.goto_pose(des_pose, use_impedance=True,
                cartesian_impedances=[2000, 2000, 1000, 100, 100, 100],)

    fa.goto_gripper(0.01)