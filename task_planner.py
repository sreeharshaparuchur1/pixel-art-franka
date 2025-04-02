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

## The first test is to go to the predefined POSE
# from there give the joint angle waypoint
# back to a pose
# go to another "ready" pose
# give the new joint angle waypoint


class SimpleColorPlanner:
    def __init__(self, home_pose, pixel_goals):
        self.home = home_pose
        self.pads = {}
        self.stamps = {}
        self.pixel_goals = pixel_goals
        self.color_order = sort_colors(self.home, self.pads)
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
        approach_pose[2, 3] += 0.05

        approach_pose = RigidTransform(
            rotation=np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]]),
            translation=approach_pose[:-1, -1],
            from_frame="franka_tool",
            to_frame="world",
        )

        self.fa.goto_pose(
            tool_pose=approach_pose,
            duration=5.0,
            use_impedance=False,  # Setting to True jitters
            cartesian_impedances=[2000, 2000, 1000, 100, 100, 100],
        )

        self.stamp(pad=True)

    # FRANKAPY goto.coordinates
    def move_to_stamp(self, color):
        stamp_pose = self.stamps[color]
        approach_pose = np.array(stamp_pose).copy()
        approach_pose[2, 3] += 0.01

        approach_pose = RigidTransform(
            rotation=np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]]),
            translation=approach_pose[:-1, -1],
            from_frame="franka_tool",
            to_frame="world",
        )

        self.fa.goto_pose(
            tool_pose=approach_pose,
            duration=5.0,
            use_impedance=False,  
            cartesian_impedances=[2000, 2000, 1000, 100, 100, 100],
        )

        self.fa.goto_gripper(0.01) # close gripper to hold the stamp

        # Retract after picking up stamp
        retract_pose = np.array(stamp_pose).copy()
        retract_pose[2, 3] += 0.05  # Retract 5 cm

        retract_pose = RigidTransform(
            rotation=np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]]),
            translation=retract_pose[:-1, -1],
            from_frame="franka_tool",
            to_frame="world",
        )

        self.fa.goto_pose(
            tool_pose=retract_pose,
            duration=5.0,
            use_impedance=True,
            cartesian_impedances=[2000, 2000, 1000, 100, 100, 100],
        )


    # FRANKAPY goto.coordinates
    def move_to_pixel(self, pixel):
        approach_pose = np.array(pixel[:-1], dtype=float).copy()
        approach_pose[2] += 0.05  # 5 cm above the pixel

        approach_pose = RigidTransform(
            rotation=np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]]),
            translation=np.array(approach_pose),
            from_frame="franka_tool",
            to_frame="world",
        )

        # linearly move down 5cm
        self.fa.goto_pose(
            tool_pose=approach_pose,
            duration=5.0,
            use_impedance=False,
            cartesian_impedances=[2000, 2000, 1000, 100, 100, 100],
        )

        # Stamp the pixel
        self.stamp(pad=False)

    # FRANKAPY Force Control
    def stamp(self, pad=True):
        current_pose = self.fa.get_pose()  # RigidTransform object

        try:
            print("Starting stamping motion...")

            desired_pose = current_pose.copy()
            desired_pose.translation -= np.array([0.0, 0.0, 0.05])  # Move down 5 cm
            print(desired_pose)

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

    # Returns order of pixel
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
            # Pick up the stamp
            self.move_to_stamp(color)

            # Dab the stamp
            self.move_to_pad(color)

            color_pixels = [p for p in self.pixel_goals if p[3] == color]
            pad_position_matrix = self.pads[color]
            pad_position = pad_position_matrix[:3, 3]
            closest_pixel = min(
                color_pixels,
                key=lambda p: np.linalg.norm(
                    np.array(pad_position) - np.array([p[0], p[1], p[2]])
                ),
            )
            ordered_pixels = self.travelling_salesman(closest_pixel[:3], color_pixels)

            for pixel in ordered_pixels:
                self.move_to_pixel(pixel)
            print(pad_position)
            plot_pixel_path(
                ordered_pixels,
                pad_position=pad_position,
                title=f"TSP Path for {color} pixels",
            )
        self.done()


# implement home distance logic
def sort_colors(home_pose, pad_poses):
    return ["R", "B", "G"]


# Bhaswanth Code
def get_stamp_pad_positions_from_aruco():
    return {"R": [0.4, -0.2, 0.03], "G": [0.6, -0.25, 0.03], "B": [0.5, -0.1, 0.03]}


# Harsha Code
def load_pixel_goals_from_image(image):
    # manually defined

    return [
        [0.436, -0.161, 0.03, "R"],
        [0.4614, -0.161, 0.03, "R"],
        [0.4868, -0.161, 0.03, "R"],
        [0.5122, -0.161, 0.03, "R"],
        [0.5376, -0.161, 0.03, "R"],
        [0.563, -0.161, 0.03, "R"],
        [0.5884, -0.161, 0.03, "R"],
        [0.6138, -0.161, 0.03, "R"],
        [0.436, -0.1356, 0.03, "G"],
        [0.4614, -0.1356, 0.03, "G"],
        [0.4868, -0.1356, 0.03, "G"],
        [0.5122, -0.1356, 0.03, "G"],
        [0.5376, -0.1356, 0.03, "G"],
        [0.563, -0.1356, 0.03, "G"],
        [0.5884, -0.1356, 0.03, "G"],
        [0.6138, -0.1356, 0.03, "G"],
        [0.436, -0.1102, 0.03, "B"],
        [0.4614, -0.1102, 0.03, "B"],
        [0.4868, -0.1102, 0.03, "B"],
        [0.5122, -0.1102, 0.03, "B"],
        [0.5376, -0.1102, 0.03, "B"],
        [0.563, -0.1102, 0.03, "B"],
        [0.5884, -0.1102, 0.03, "B"],
        [0.6138, -0.1102, 0.03, "B"],
    ]


def plot_pixel_path(pixel_list, pad_position=None, title="Pixel Path"):
    if not pixel_list:
        print("Empty pixel list.")
        return

    print("Plotting pixel path.....")

    xs = [p[0] for p in pixel_list]
    ys = [p[1] for p in pixel_list]
    colors = [p[3] for p in pixel_list]

    plt.figure(figsize=(8, 6))

    plt.plot(xs, ys, "-k", linewidth=1, label="TSP path")

    for x, y, c in zip(xs, ys, colors):
        plt.plot(x, y, "o", color="red", markersize=6)
        plt.text(x + 0.002, y + 0.002, c, fontsize=8)

    if pad_position is not None:
        px, py, _ = pad_position
        plt.plot(px, py, "s", color="blue", markersize=10, label="Stamp Pad")
        plt.text(px + 0.002, py + 0.002, "Pad", fontsize=9, color="blue")

    plt.title(title)
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.savefig("pixel_path_plot.png", dpi=300)
    plt.close()
    print("Plot saved as 'pixel_path_plot.png'")


def main():
    home_pose = np.array(
        [[1, 0, 0, 0.5], [0, 1, 0, 0.0], [0, 0, 1, 0.4], [0, 0, 0, 1.0]]
    )
    # pad_poses = get_stamp_pad_positions_from_aruco()
    pixel_goals = load_pixel_goals_from_image("pixel_art.png")
    planner = SimpleColorPlanner(home_pose, pixel_goals)

    rospy.loginfo("Waiting for pad poses...")
    while not rospy.is_shutdown() and len(planner.pads) < 4:
        rospy.sleep(0.1)

    rospy.loginfo("All pad poses received. Starting run...")

    planner.run()


if __name__ == "__main__":
    main()
