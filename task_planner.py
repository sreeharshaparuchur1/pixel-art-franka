import numpy as np
import networkx as nx
import matplotlib.pyplot as plt

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
    def travelling_salesman(self, start_pose, pixels):
        G = nx.Graph()
        points = [(p[0], p[1], p[2]) for p in pixels]

        # ad nodes 
        for i, pt in enumerate(points):
            G.add_node(i, pos=pt)

        # add edge weights (euclidean distance)
        for i in range(len(points)):
            for j in range(i + 1, len(points)):
                dist = np.linalg.norm(np.array(points[i]) - np.array(points[j]))
                G.add_edge(i, j, weight=dist)

        start_point = start_pose[:3, 3]
        start_idx = np.argmin([np.linalg.norm(start_point - np.array(p)) for p in points])
        # tsp_path = nx.approximation.traveling_salesman_problem(G, cycle=False, weight='weight', nodes=range(len(points)), start=start_idx)
        tsp_path = nx.approximation.greedy_tsp(G, weight='weight', source=start_idx)

        return [pixels[i] for i in tsp_path]


    def run(self):
        for color in self.color_order:
            # Dab the stamp
            self.move_to_pad(color)
            self.stamp()

            # 
            color_pixels = [p for p in self.pixel_goals if p[3] == color]
            start_pose = self.pads[color]
            ordered_pixels = self.travelling_salesman(start_pose, color_pixels)

            for pixel in ordered_pixels:
                self.move_to_pixel(pixel)
                self.stamp()
            # plot_pixel_path(ordered_pixels, title=f"TSP Path for {color} pixels")
        self.done()

# implement home distance logic
def sort_colors(home_pose, pad_poses):
    return ['R', 'B', 'G']

# Bhaswanth Code
def get_stamp_pad_poses_from_aruco():
    pads = {
        'R': np.array([[1, 0, 0, 0.4],
                       [0, 1, 0, -0.2],
                       [0, 0, 1, 0.03],
                       [0, 0, 0, 1]]),
        'G': np.array([[1, 0, 0, 0.6],
                       [0, 1, 0, -0.25],
                       [0, 0, 1, 0.03],
                       [0, 0, 0, 1]]),
        'B': np.array([[1, 0, 0, 0.5],
                       [0, 1, 0, -0.1],
                       [0, 0, 1, 0.03],
                       [0, 0, 0, 1]])
    }
    return pads

# harsha Code
def load_pixel_goals_from_image(image):
    # manually defined 
    return [
    [0.48, -0.16, 0.03, 'R'],
    [0.50, -0.16, 0.03, 'R'],
    [0.52, -0.16, 0.03, 'R'],
    [0.50, -0.14, 0.03, 'G'],
    [0.52, -0.14, 0.03, 'G'],
    [0.54, -0.12, 0.03, 'B'],
]

def plot_pixel_path(pixel_list, title="Pixel Path"):
    """
    pixel_list: ordered list of [x, y, z, color]
    """
    if not pixel_list:
        print("Empty pixel list.")
        return

    # Extract coordinates
    xs = [p[0] for p in pixel_list]
    ys = [p[1] for p in pixel_list]
    colors = [p[3] for p in pixel_list]

    # Draw path
    plt.figure(figsize=(8, 6))
    plt.plot(xs, ys, '-k', linewidth=1)  # path lines

    # Draw points
    for x, y, c in zip(xs, ys, colors):
        plt.plot(x, y, 'o', color='red', markersize=6)
        plt.text(x + 0.002, y + 0.002, c, fontsize=8)

    plt.title(title)
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.axis('equal')
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def main():
    home_pose = np.array([[1, 0, 0, 0.5],
                          [0, 1, 0, 0.0],
                          [0, 0, 1, 0.4],
                          [0, 0, 0, 1.0]  ])
    pad_poses = get_stamp_pad_poses_from_aruco()
    pixel_goals = load_pixel_goals_from_image("pixel_art.png")
    planner = SimpleColorPlanner(home_pose, pad_poses, pixel_goals)
    planner.run()


if __name__ == "__main__":
    main()

