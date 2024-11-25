import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import robotic_rotation as rr

def plot_quaternion(quaternion, ax=None):
    """
    Plot a quaternion in 3D space as XYZ axes in a coordinate system.

    Args:
        quaternion: A list or numpy array of shape (4,), representing the quaternion [w, x, y, z].
        ax: A matplotlib 3D axis object. If None, a new figure and axis will be created.
    """
    # Convert quaternion to rotation matrix
    rotation_matrix = rr.quat2rotm(quaternion)  # Quaternion format [x, y, z, w]

    # Define origin
    origin = np.array([0, 0, 0])

    # Define unit axes
    x_axis = rotation_matrix[:, 0]  # X-axis in rotated frame
    y_axis = rotation_matrix[:, 1]  # Y-axis in rotated frame
    z_axis = rotation_matrix[:, 2]  # Z-axis in rotated frame

    # Create a new plot if no axis is provided
    if ax is None:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_box_aspect([1, 1, 1])

    # Plot the axes
    ax.quiver(*origin, *x_axis, color='r', label='X-axis', length=1, normalize=True)
    ax.quiver(*origin, *y_axis, color='g', label='Y-axis', length=1, normalize=True)
    ax.quiver(*origin, *z_axis, color='b', label='Z-axis', length=1, normalize=True)

    # Set labels and axis limits
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Add legend
    ax.legend()