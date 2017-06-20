import numpy as np
import seaborn as sns

from plotters import accuracy_heatmap

# Seaborn configuration
sns.set(context="paper", font="monospace", font_scale=1.5)
sns.set_style("dark")

# Input filename
filename = "./data/accuracy.dat"

# Output path
out_path = "./output/"

# Load the data into different arrays
radii, add_vert, connects, remov_vert, add_edges, remov_edges, num_walks, walk_len, accuracy = \
    np.loadtxt(filename, unpack=True, dtype='int,int,int,int,int,int,int,int,float')

# Extract unique values for the arrays and the respective number of different elements
radius_list = np.unique(radii)
num_radii = len(radius_list)

for r in radius_list:
    title = "num-walks-vs-walk-length-with-radius-{}".format(r)
    accuracy_heatmap(num_walks, walk_len, accuracy, radii, r, title, "walk length", "number of random walks")

for r in radius_list:
    title = "num-new-vertices-vs-num-new-edges-with-radius-{}".format(r)
    accuracy_heatmap(add_vert, add_edges, accuracy, radii, r, title, "added edges", "added vertices")



