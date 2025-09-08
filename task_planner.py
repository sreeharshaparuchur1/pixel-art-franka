#!/usr/bin/env python3

import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
from networkx.algorithms.approximation import christofides
import rospy
from std_msgs.msg import String, Float32MultiArray
from frankapy import FrankaArm
from autolab_core import RigidTransform
from time import time, sleep
import pdb
from robot_constants import RobotConstants, ColorConstants
from visualization import plot_pixel_path

class SimpleColorPlanner:
    def __init__(self, home_pose, pixel_goals):
        self.home = home_pose
        self.pads = {}
        self.stamps = {}
        self.pixel_goals = pixel_goals
        self.color_order = ColorConstants.DEFAULT_COLOR_ORDER
        self.current_color_idx = 0

        self.fa = FrankaArm()

        rospy.Subscriber("/pad_pose_array", Float32MultiArray, self.pad_pose_callback)
        rospy.Subscriber(
            "/stamp_pose_array", Float32MultiArray, self.stamp_pose_callback
        )

    def pad_pose_callback(self, msg):

        # message containing the poses and colors of the pads.
        data = msg.data
        colors = ["R", "G", "B", "K"]
        for i in range(4):
            pose_values = data[i * 16 : (i + 1) * 16]
            pose_matrix = np.array(pose_values).reshape(4, 4)
            self.pads[colors[i]] = pose_matrix
            # print(f"Updated pad pose for color {colors[i]}: {pose_matrix}")

    def stamp_pose_callback(self, msg):

        # message containing the poses and colors of the stamps.
        data = msg.data
        colors = ["R", "G", "B", "K"]
        for i in range(4):
            pose_values = data[i * 16 : (i + 1) * 16]
            pose_matrix = np.array(pose_values).reshape(4, 4)
            self.stamps[colors[i]] = pose_matrix
            # print(f"Updated stamp pose for color {colors[i]}: {pose_matrix}")

    # FRANKAPY goto.coordinates
    def move_to_pad(self, color):
        pad_pose = self.pads[color]
        approach_pose = np.array(pad_pose).copy()
        approach_pose[2, 3] += RobotConstants.PAD_APPROACH_HEIGHT

        approach_pose = RigidTransform(
            rotation=RobotConstants.STANDARD_ROTATION,
            translation=approach_pose[:-1, -1],
            from_frame="franka_tool",
            to_frame="world",
        )

        self.fa.goto_pose(
            tool_pose=approach_pose,
            duration=RobotConstants.SLOW_MOVEMENT_DURATION,
            use_impedance=False,  # Setting to True jitters
            cartesian_impedances=RobotConstants.DEFAULT_IMPEDANCES,
        )

        self.stamp(pad=True)

    # FRANKAPY goto.coordinates
    def move_to_stamp(self, color):
        print(f"Moving to stamp color {color}")
        # self.fa.goto_gripper(0.08)
        self.fa.open_gripper()

        stamp_pose = self.stamps[color]
        approach_pose = np.array(stamp_pose).copy()
        approach_pose[2, 3] += RobotConstants.STAMP_APPROACH_HEIGHT

        approach_pose = RigidTransform(
            rotation=RobotConstants.STANDARD_ROTATION,
            translation=approach_pose[:-1, -1],
            from_frame="franka_tool",
            to_frame="world",
        )

        self.fa.goto_pose(
            tool_pose=approach_pose,
            duration=RobotConstants.SLOW_MOVEMENT_DURATION,
            use_impedance=False,
            cartesian_impedances=RobotConstants.DEFAULT_IMPEDANCES,
        )

        self.fa.goto_gripper(RobotConstants.GRIPPER_CLOSED)  # close gripper to hold the stamp

        # final_pose = self.fa.get_pose()
        # if np.linalg.norm(final_pose.translation - approach_pose.translation) < 0.005:
        #     self.fa.goto_gripper(0.01)
        # else:
        #     rospy.logwarn("Robot hasn't reached the pose yet. Waiting...")
        #     rospy.sleep(1.0)
        #     self.fa.goto_gripper(0.01)

        # Retract after picking up stamp
        retract_pose = np.array(stamp_pose).copy()
        retract_pose[2, 3] += RobotConstants.RETRACT_HEIGHT

        retract_pose = RigidTransform(
            rotation=RobotConstants.STANDARD_ROTATION,
            translation=retract_pose[:-1, -1],
            from_frame="franka_tool",
            to_frame="world",
        )

        self.fa.goto_pose(
            tool_pose=retract_pose,
            duration=RobotConstants.FAST_MOVEMENT_DURATION,
            use_impedance=True,
            cartesian_impedances=RobotConstants.DEFAULT_IMPEDANCES,
        )

    def return_to_stamp(self, color):
        print(f"Returning stamp for color {color}")

        stamp_pose = self.stamps[color]
        approach_pose = np.array(stamp_pose).copy()
        approach_pose[2, 3] += 0.08

        approach_pose = RigidTransform(
            rotation=np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]]),
            translation=approach_pose[:-1, -1],
            from_frame="franka_tool",
            to_frame="world",
        )

        self.fa.goto_pose(
            tool_pose=approach_pose,
            duration=10.0,
            use_impedance=False,
            cartesian_impedances=[2000, 2000, 1000, 100, 100, 100],
        )

        # Place stamp
        place_pose = np.array(stamp_pose).copy()
        place_pose[2, 3] += 0.01  # Go down to 1 cm

        place_pose = RigidTransform(
            rotation=np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]]),
            translation=place_pose[:-1, -1],
            from_frame="franka_tool",
            to_frame="world",
        )

        self.fa.goto_pose(
            tool_pose=place_pose,
            duration=5.0,
            use_impedance=True,
            cartesian_impedances=[2000, 2000, 1000, 100, 100, 100],
        )

        # self.fa.goto_gripper(0.08, speed=0.1)
        self.fa.open_gripper()
        # print("Opening gripper at stamp home position")
        for _ in range(50):
            if self.fa.get_gripper_width() > 0.07:
                break
            sleep(0.05)
        else:
            print("Gripper didnt open fully!")

        # Retract after placing the stamp
        # self.fa.goto_pose(
        #     tool_pose=approach_pose,
        #     duration=5.0,
        #     use_impedance=True,
        #     cartesian_impedances=[2000, 2000, 1000, 100, 100, 100],
        # )

        self.fa.open_gripper()

    # FRANKAPY goto.coordinates
    def move_to_pixel(self, pixel):
        approach_pose = np.array(pixel[:-1], dtype=float).copy()
        approach_pose[2] += 0.02  # 2 cm above the pixel

        approach_pose = RigidTransform(
            rotation=np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]]),
            translation=np.array(approach_pose),
            from_frame="franka_tool",
            to_frame="world",
        )

        self.fa.goto_pose(
            tool_pose=approach_pose,
            duration=5.0,
            use_impedance=False,
            cartesian_impedances=[2000, 2000, 1000, 100, 100, 100],
        )

        # Stamp the pixel
        self.stamp(pad=False)

    def stamp(self, pad=True):
        current_pose = self.fa.get_pose()  # RigidTransform object

        try:
            print("Starting stamping motion...")

            desired_pose = current_pose.copy()
            desired_pose.translation -= np.array([0.0, 0.0, 0.05])  # Move down 5 cm

            self.fa.goto_pose(
                tool_pose=desired_pose,
                duration=5.0,
                use_impedance=True,
                cartesian_impedances=[2000, 2000, 1000, 100, 100, 100],
            )

            current_pose = self.fa.get_pose()  # update current pose reference

            print("Stamp applied. Retracting...")

            # Retract after stamp
            retract_pose = current_pose.copy()
            retract_pose.translation += np.array([0.0, 0.0, 0.05])  # Retract 5 cm

            self.fa.goto_pose(
                tool_pose=retract_pose,
                duration=5.0,
                use_impedance=True,
                cartesian_impedances=[2000, 2000, 1000, 100, 100, 100],
            )

            print("Stamping completed.")

        except Exception as e:
            print(f"Stamping interrupted: {e}")

    def done(self):
        print("All pixels stamped.")

    def travelling_salesman(self, start_position, pixels):
        G = nx.Graph()
        points = [(p[0], p[1], p[2]) for p in pixels]

        for i, pt in enumerate(points):
            G.add_node(i, pos=pt)

        for i in range(len(points)):
            for j in range(i + 1, len(points)):
                dist = np.linalg.norm(np.array(points[i]) - np.array(points[j]))
                G.add_edge(i, j, weight=dist)

        start_idx = np.argmin(
            [np.linalg.norm(np.array(start_position) - np.array(p)) for p in points]
        )
        tsp_cycle = christofides(G, weight="weight")
        start_pos = tsp_cycle.index(start_idx)
        tsp_path = tsp_cycle[start_pos:] + tsp_cycle[:start_pos]

        return [pixels[i] for i in tsp_path]

    def run(self):
        for color in self.color_order:
            color_pixels = [p for p in self.pixel_goals if p[3] == color]
            if len(color_pixels) == 0:
                print(f"There's no colour {color} in the image!")
                continue
            # Pick up the stamp
            self.move_to_stamp(color)

            # Dab the stamp
            self.move_to_pad(color)

            pad_position_matrix = self.pads[color]
            pad_position = pad_position_matrix[:3, 3]
            closest_pixel = min(
                color_pixels,
                key=lambda p: np.linalg.norm(
                    np.array(pad_position) - np.array([p[0], p[1], p[2]])
                ),
            )
            ordered_pixels = self.travelling_salesman(closest_pixel[:3], color_pixels)

            stamp_counter = 0

            for pixel in ordered_pixels:
                self.move_to_pixel(pixel)

                stamp_counter += 1

                if stamp_counter % RobotConstants.REDAB_FREQUENCY == 0 and stamp_counter != len(ordered_pixels):
                    print(f"Re-dabbing stamp for {color} after {stamp_counter} pixels.")
                    self.move_to_pad(color)

            print(pad_position)

            self.return_to_stamp(color)  # Return the stamp after finishing color
            self.fa.open_gripper()

            plot_pixel_path(
                ordered_pixels,
                pad_position=pad_position,
                title=f"TSP Path for {color} pixels",
            )

        self.done()


def load_pixel_goals_from_image(image):
    # manually defined

    # return [
    #     [0.436, -0.161, 0.03, 'B'], [0.436, -0.1356, 0.03, 'G'], [0.436, -0.1102, 0.03, 'G'], [0.436, -0.0848, 0.03, 'B'],
    #     [0.436, -0.0594, 0.03, 'G'], [0.436, -0.034, 0.03, 'G'], [0.436, -0.0086, 0.03, 'B'], [0.436, 0.0168, 0.03, 'G'],
    #     [0.4614, -0.161, 0.03, 'G'], [0.4614, -0.1356, 0.03, 'K'], [0.4614, -0.1102, 0.03, 'K'], [0.4614, -0.0848, 0.03, 'B'],
    #     [0.4614, -0.0594, 0.03, 'G'], [0.4614, -0.034, 0.03, 'K'], [0.4614, -0.0086, 0.03, 'K'], [0.4614, 0.0168, 0.03, 'G'],
    #     [0.4868, -0.161, 0.03, 'G'], [0.4868, -0.1356, 0.03, 'K'], [0.4868, -0.1102, 0.03, 'K'], [0.4868, -0.0848, 0.03, 'G'],
    #     [0.4868, -0.0594, 0.03, 'G'], [0.4868, -0.034, 0.03, 'K'], [0.4868, -0.0086, 0.03, 'K'], [0.4868, 0.0168, 0.03, 'G'],
    #     [0.5122, -0.161, 0.03, 'B'], [0.5122, -0.1356, 0.03, 'G'], [0.5122, -0.1102, 0.03, 'G'], [0.5122, -0.0848, 0.03, 'K'],
    #     [0.5122, -0.0594, 0.03, 'K'], [0.5122, -0.034, 0.03, 'G'], [0.5122, -0.0086, 0.03, 'G'], [0.5122, 0.0168, 0.03, 'B'],
    #     [0.5376, -0.161, 0.03, 'G'], [0.5376, -0.1356, 0.03, 'G'], [0.5376, -0.1102, 0.03, 'K'], [0.5376, -0.0848, 0.03, 'K'],
    #     [0.5376, -0.0594, 0.03, 'K'], [0.5376, -0.034, 0.03, 'K'], [0.5376, -0.0086, 0.03, 'G'], [0.5376, 0.0168, 0.03, 'G'],
    #     [0.563, -0.161, 0.03, 'B'], [0.563, -0.1356, 0.03, 'G'], [0.563, -0.1102, 0.03, 'K'], [0.563, -0.0848, 0.03, 'K'],
    #     [0.563, -0.0594, 0.03, 'K'], [0.563, -0.034, 0.03, 'K'], [0.563, -0.0086, 0.03, 'G'], [0.563, 0.0168, 0.03, 'G'],
    #     [0.5884, -0.161, 0.03, 'G'], [0.5884, -0.1356, 0.03, 'G'], [0.5884, -0.1102, 0.03, 'K'], [0.5884, -0.0848, 0.03, 'G'],
    #     [0.5884, -0.0594, 0.03, 'G'], [0.5884, -0.034, 0.03, 'K'], [0.5884, -0.0086, 0.03, 'B'], [0.5884, 0.0168, 0.03, 'G'],
    #     [0.6138, -0.161, 0.03, 'G'], [0.6138, -0.1356, 0.03, 'G'], [0.6138, -0.1102, 0.03, 'B'], [0.6138, -0.0848, 0.03, 'G'],
    #     [0.6138, -0.0594, 0.03, 'B'], [0.6138, -0.034, 0.03, 'G'], [0.6138, -0.0086, 0.03, 'G'], [0.6138, 0.0168, 0.03, 'G']
    # ]

    return [
        [0.436, -0.161, 0.03, "B"],
        [0.436, -0.1356, 0.03, "G"],
        [0.436, -0.1102, 0.03, "G"],
        [0.436, -0.0594, 0.03, "G"],
        [0.436, -0.034, 0.03, "G"],
        [0.436, -0.0086, 0.03, "B"],
        [0.436, 0.0168, 0.03, "G"],
        [0.4614, -0.161, 0.03, "G"],
        [0.4614, -0.1356, 0.03, "K"],
        [0.4614, -0.1102, 0.03, "K"],
        [0.4614, -0.0594, 0.03, "G"],
        [0.4614, -0.034, 0.03, "K"],
        [0.4614, -0.0086, 0.03, "K"],
        [0.4614, 0.0168, 0.03, "G"],
        [0.4868, -0.161, 0.03, "G"],
        [0.4868, -0.1356, 0.03, "K"],
        [0.4868, -0.1102, 0.03, "K"],
        [0.4868, -0.0848, 0.03, "G"],
        [0.4868, -0.0594, 0.03, "G"],
        [0.4868, -0.034, 0.03, "K"],
        [0.4868, -0.0086, 0.03, "K"],
        [0.4868, 0.0168, 0.03, "G"],
        [0.5122, -0.1356, 0.03, "G"],
        [0.5122, -0.1102, 0.03, "G"],
        [0.5122, -0.0848, 0.03, "K"],
        [0.5122, -0.0594, 0.03, "K"],
        [0.5122, -0.034, 0.03, "G"],
        [0.5122, -0.0086, 0.03, "G"],
        [0.5376, -0.161, 0.03, "G"],
        [0.5376, -0.1356, 0.03, "G"],
        [0.5376, -0.1102, 0.03, "K"],
        [0.5376, -0.0848, 0.03, "K"],
        [0.5376, -0.0594, 0.03, "K"],
        [0.5376, -0.034, 0.03, "K"],
        [0.5376, -0.0086, 0.03, "G"],
        [0.5376, 0.0168, 0.03, "G"],
        [0.563, -0.1356, 0.03, "G"],
        [0.563, -0.1102, 0.03, "K"],
        [0.563, -0.0848, 0.03, "K"],
        [0.563, -0.0594, 0.03, "K"],
        [0.563, -0.034, 0.03, "K"],
        [0.563, -0.0086, 0.03, "G"],
        [0.563, 0.0168, 0.03, "G"],
        [0.5884, -0.161, 0.03, "G"],
        [0.5884, -0.1356, 0.03, "G"],
        [0.5884, -0.1102, 0.03, "K"],
        [0.5884, -0.0848, 0.03, "G"],
        [0.5884, -0.0594, 0.03, "G"],
        [0.5884, -0.034, 0.03, "K"],
        [0.5884, 0.0168, 0.03, "G"],
        [0.6138, -0.161, 0.03, "G"],
        [0.6138, -0.1356, 0.03, "G"],
        [0.6138, -0.0848, 0.03, "G"],
        [0.6138, -0.034, 0.03, "G"],
        [0.6138, -0.0086, 0.03, "G"],
        [0.6138, 0.0168, 0.03, "G"],
    ]


