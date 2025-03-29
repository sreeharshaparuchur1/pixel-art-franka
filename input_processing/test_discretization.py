import numpy as np

vals = np.load("grid_locations_colour.npy", allow_pickle=True)
# vals = np.load("grid_locations_colour_t2.npy", allow_pickle=True)
# vals = np.load("grid_locations_colour_t3.npy", allow_pickle=True)

# print(vals.shape)
for row in range(vals.shape[0]):
    for col in range(vals.shape[1]):
        print(f"{vals[col][row][-1]}", end=" ")
    print(f"")