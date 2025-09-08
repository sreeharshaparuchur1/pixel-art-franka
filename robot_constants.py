#!/usr/bin/env python3
import numpy as np

class RobotConstants:
    
    # Standard end-effector orientation (180° rotation around X-axis)
    STANDARD_ROTATION = np.array([
        [1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 0.0, -1.0]
    ])
    
    # Default cartesian impedance values for stable movement
    DEFAULT_IMPEDANCES = [2000, 2000, 1000, 100, 100, 100]
    
    STAMP_APPROACH_HEIGHT = 0.01  # 1 cm above stamp
    PAD_APPROACH_HEIGHT = 0.02    # 2 cm above pad
    PIXEL_APPROACH_HEIGHT = 0.02  # 2 cm above pixel
    RETRACT_HEIGHT = 0.05         # 5 cm retract after operations
    RETURN_APPROACH_HEIGHT = 0.08 # 8 cm approach for returning stamps
    
    # Gripper positions
    GRIPPER_CLOSED = 0.01
    GRIPPER_OPEN = 0.08
    
    FAST_MOVEMENT_DURATION = 5.0
    SLOW_MOVEMENT_DURATION = 10.0
    
    STAMP_DOWN_DISTANCE = 0.05
    STAMP_UP_DISTANCE = 0.05
    
    # Re-dab frequency
    REDAB_FREQUENCY = 3


class ColorConstants:
    
    COLORS = ["R", "G", "B", "K"]  # Red, Green, Blue, Black
    DEFAULT_COLOR_ORDER = ["R", "B", "G", "K"]