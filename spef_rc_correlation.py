import numpy as np
import scipy.spatial

# Your existing code


def _update_plot():
    # Use numpy and scipy to build kdtrees
    kdtree_c = scipy.spatial.cKDTree(data_c)
    kdtree_r = scipy.spatial.cKDTree(data_r)
    # ... rest of the update plot code


def _on_motion(event):
    # Use the kdtree to efficiently query hover data
    point = np.array([event.xdata, event.ydata])
    idx = kdtree.query(point)
    # ... rest of the on motion code

# Replace scatter with ax.plot
ax.plot(x_data, y_data, linestyle='', marker='.', rasterized=True)