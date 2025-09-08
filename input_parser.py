import cv2
import numpy as np

# Define color ranges (in HSV format)
COLOR_RANGES = {
    'R': ((0, 120, 70), (10, 255, 255)),  # Red
    'B': ((65, 150, 0), (140, 255, 255)),  # Blue
    'G': ((40, 40, 40), (50, 255, 255)),  # Green
    'O': ((10, 100, 20), (35, 255, 255)),  # Orange
    'K': ((0, 0, 0), (180, 50, 100)),  # Black
}

Z_VALUE = 0.03  # Constant Z height in meters
INCH_TO_METER = 0.0254  # Conversion factor
FRANKA_BASE_OFFSET_X = 0.436
FRANKA_BASE_OFFSET_Y = -0.161


def classify_color(region):
    """Classifies the dominant color in the given region."""
    hsv = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)
    
    # rgb = cv2.cvtColor(region, cv2.COLOR_BGR2RGB)

    label_ret = 'W'
    for label, (lower, upper) in COLOR_RANGES.items():
        mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
        # print(f"mask shape: {mask.shape} \n {mask}")

        if np.sum(mask) > (255 * 28 * 2):  # If there are at least 5 rows (8 - 3, 8) that are in the query range
            # print(f"how many pixels match the label: {np.sum(mask)} and label: {label}")
            # return label
            label_ret = label
    # return 'W'
    return label_ret

def process_image(img_path, output_npy="grid_locations_colour.npy"):
# def process_image(img_path, output_npy="grid_locations_colour_t3.npy"):
# def process_image(img_path, output_npy="grid_locations_colour_t2.npy"):
    """Reads an image, divides it into an 8x8 grid, classifies each region, and saves results as a NumPy array."""
    img = cv2.imread(img_path)
    if img is None:
        print("\n!!")
        return
    
    h, w, _ = img.shape
    grid_h, grid_w = h // 8, w // 8  # Divide into 8x8 grid
    # Assume each grid cell is 1 inch x 1 inch
    cell_size_in_meters = 1 * INCH_TO_METER
    data = []  # List of lists [x, y, z, color]

    for i in range(8): # y in robot coordinate frame
        row = []
        for j in range(8): # x in robot coordinate frame
            x_pixel, y_pixel = j * grid_w, i * grid_h
            region = img[x_pixel:x_pixel + grid_w, y_pixel:y_pixel + grid_h]
            color = classify_color(region)
            
            x_meters = j * cell_size_in_meters + FRANKA_BASE_OFFSET_X
            y_meters = i * cell_size_in_meters + FRANKA_BASE_OFFSET_Y

            row.append([x_meters, y_meters, Z_VALUE, color])
        
        data.append(row)

    # Save to a NumPy .npy file
    np.save(output_npy, np.array(data, dtype=object))
    print(f"Grid data saved to {output_npy}")

if __name__ == "__main__":
    process_image('/home/harry/cmu/spring_25/16662/franka_ros2/data/test_mvp.png') 
    # process_image('/home/harry/cmu/spring_25/16662/franka_ros2/data/test_two_mvp.png') 
    # process_image('/home/harry/cmu/spring_25/16662/franka_ros2/data/test_three_mvp.png') 


