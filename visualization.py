#!/usr/bin/env python3

import matplotlib.pyplot as plt


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