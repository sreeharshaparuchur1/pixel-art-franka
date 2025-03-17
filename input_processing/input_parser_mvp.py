#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import String
from cv_bridge import CvBridge

# Define color ranges (in HSV format)
COLOR_RANGES = {
    'R': ((0, 120, 70), (10, 255, 255)),  # Red (lower)
    'B': ((100, 150, 0), (140, 255, 255)),  # Blue
    'G': ((40, 40, 40), (90, 255, 255)),  # Green
    'O': ((10, 100, 20), (25, 255, 255)),  # Orange
    'K': ((0, 0, 0), (180, 255, 50)),  # Black (low brightness)
}

def classify_color(region):
    """ Classifies the dominant color in the given region. """
    hsv = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)
    
    for label, (lower, upper) in COLOR_RANGES.items():
        mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
        if np.sum(mask) > 0:  # If any pixel matches
            return label
    return 'U'  # Unknown

def process_image(img_path):
    """ Reads an image, divides it into 8x8 grid, and classifies each region. """
    img = cv2.imread(img_path)
    if img is None:
        rospy.logerr("Failed to load image!")
        return
    
    h, w, _ = img.shape
    grid_h, grid_w = h // 8, w // 8

    grid_matrix = []
    pub = rospy.Publisher('/grid_colors', String, queue_size=10)
    rospy.init_node('image_grid_classifier', anonymous=True)
    rate = rospy.Rate(1)  # 1 Hz
    
    for i in range(8):
        row = []
        for j in range(8):
            x, y = j * grid_w, i * grid_h
            region = img[y:y + grid_h, x:x + grid_w]
            color = classify_color(region)
            row.append(color)
            
            # Convert to robot base frame (Assume function transform_to_base exists)
            base_x, base_y = transform_to_base(x, y)
            
            msg = f"{base_x},{base_y},{color}"
            pub.publish(msg)
        
        grid_matrix.append(row)

    for row in grid_matrix:
        print(" ".join(row))
    
    rate.sleep()

def transform_to_base(x, y):
    """ Placeholder for transformation from camera frame to base frame. """
    # TODO: Use TF2 or a known transformation to convert coordinates.
    return x, y

if __name__ == "__main__":
    try:
        process_image('/path/to/your/image.jpg')
    except rospy.ROSInterruptException:
        pass
