import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
from networkx.algorithms.approximation import christofides

class SimpleColorPlanner:
    def __init__(self, home_pose, pad_poses, pixel_goals):
        self.home = home_pose
        self.pads = pad_poses
        self.pixel_goals = pixel_goals
        self.color_order = sort_colors(self.home, self.pads)
        self.current_color_idx = 0

    # FRANKAPY goto.coordinates
    def move_to_pad(self, color):
        print(f"moving to color {color}")

    # FRANKAPY goto.coordinates
    def move_to_pixel(self, pixel):
        print(f"moving to pixel {pixel}")

    # FRANKAPY Force Control
    def stamp(self):
        print(f"Stamping")

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

        start_idx = np.argmin([np.linalg.norm(np.array(start_position) - np.array(p)) for p in points])
        tsp_cycle = christofides(G, weight='weight')
        start_pos = tsp_cycle.index(start_idx)
        tsp_path = tsp_cycle[start_pos:] + tsp_cycle[:start_pos]

        return [pixels[i] for i in tsp_path]
    
    def run(self):
        for color in self.color_order:
            # Dab the Stamp
            self.move_to_pad(color)
            self.stamp()

            color_pixels = [p for p in self.pixel_goals if p[3] == color]
            pad_position = self.pads[color]
            closest_pixel = min(color_pixels, key=lambda p: np.linalg.norm(np.array(pad_position) - np.array([p[0], p[1], p[2]])))
            ordered_pixels = self.travelling_salesman(closest_pixel[:3], color_pixels)

            for pixel in ordered_pixels:
                self.move_to_pixel(pixel)
                self.stamp()
            plot_pixel_path(ordered_pixels, pad_position=pad_position, title=f"TSP Path for {color} pixels")
        self.done()

# implement home distance logic
def sort_colors(home_pose, pad_poses):
    return ['R', 'B', 'G']

# Bhaswanth Code
def get_stamp_pad_positions_from_aruco():
    return {
        'R': [0.4, -0.2, 0.03],
        'G': [0.6, -0.25, 0.03],
        'B': [0.5, -0.1, 0.03]
    }

# harsha Code
def load_pixel_goals_from_image(image):
    # manually defined 
    return [
    [0.48, -0.16, 0.03, 'R'],
    [0.51, -0.15, 0.03, 'R'],
    [0.49, -0.13, 0.03, 'R'],
    [0.53, -0.17, 0.03, 'R'],
    [0.50, -0.12, 0.03, 'R'],
    [0.52, -0.14, 0.03, 'R'],
    [0.47, -0.15, 0.03, 'R'],
    [0.54, -0.13, 0.03, 'R'],
    [0.49, -0.17, 0.03, 'R'],
    [0.51, -0.11, 0.03, 'R'],
    [0.53, -0.11, 0.03, 'R'],
    [0.55, -0.15, 0.03, 'R'],

    [0.60, -0.25, 0.03, 'G'],
    [0.62, -0.24, 0.03, 'G'],
    [0.58, -0.26, 0.03, 'G'],
    [0.59, -0.23, 0.03, 'G'],
    [0.61, -0.27, 0.03, 'G'],
    [0.63, -0.25, 0.03, 'G'],
    [0.60, -0.22, 0.03, 'G'],
    [0.62, -0.23, 0.03, 'G'],

    [0.50, -0.10, 0.03, 'B'],
    [0.52, -0.09, 0.03, 'B'],
    [0.48, -0.11, 0.03, 'B'],
    [0.51, -0.08, 0.03, 'B'],
    [0.49, -0.09, 0.03, 'B'],
    [0.53, -0.10, 0.03, 'B'],
    [0.47, -0.10, 0.03, 'B'],
    [0.52, -0.07, 0.03, 'B']
    ]

def plot_pixel_path(pixel_list, pad_position=None, title="Pixel Path"):
    if not pixel_list:
        print("Empty pixel list.")
        return

    xs = [p[0] for p in pixel_list]
    ys = [p[1] for p in pixel_list]
    colors = [p[3] for p in pixel_list]

    plt.figure(figsize=(8, 6))

    plt.plot(xs, ys, '-k', linewidth=1, label="TSP path")

    for x, y, c in zip(xs, ys, colors):
        plt.plot(x, y, 'o', color='red', markersize=6)
        plt.text(x + 0.002, y + 0.002, c, fontsize=8)

    if pad_position:
        px, py, _ = pad_position
        plt.plot(px, py, 's', color='blue', markersize=10, label="Stamp Pad")
        plt.text(px + 0.002, py + 0.002, 'Pad', fontsize=9, color='blue')

    plt.title(title)
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

def main():
    home_pose = np.array([[1, 0, 0, 0.5],
                          [0, 1, 0, 0.0],
                          [0, 0, 1, 0.4],
                          [0, 0, 0, 1.0]  ])
    pad_poses = get_stamp_pad_positions_from_aruco()
    pixel_goals = load_pixel_goals_from_image("pixel_art.png")
    planner = SimpleColorPlanner(home_pose, pad_poses, pixel_goals)
    planner.run()


if __name__ == "__main__":
    main()

