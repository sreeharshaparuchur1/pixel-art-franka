#!/usr/bin/env python3
"""
Main entry point for the system.
"""

import numpy as np
import rospy
from task_planner import SimpleColorPlanner, load_pixel_goals_from_image


def main():
    home_pose = np.array(
        [[1, 0, 0, 0.5], 
         [0, 1, 0, 0.0], 
         [0, 0, 1, 0.4], 
         [0, 0, 0, 1.0]]
    )
    
    # Load pixel goals from image processing
    pixel_goals = load_pixel_goals_from_image("pixel_art.png")
    
    # Initialize the task planner
    planner = SimpleColorPlanner(home_pose, pixel_goals)

    rospy.loginfo("Waiting for pad poses...")
    while not rospy.is_shutdown() and len(planner.pads) < 4:
        rospy.sleep(0.1)

    rospy.loginfo("All pad poses received. Starting run...")
    planner.run()


if __name__ == "__main__":
    main()