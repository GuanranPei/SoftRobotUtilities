import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot_single_sections(S, Deltax, Deltay):
    """
    Plots soft robotic sections using segmented linear trajectory.

    Args:
        S: 2D numpy array, trunk length vector of 3 sections.
        Deltax: 2D numpy array, new coordinate vector in configuration space of 3 sections (x-axis).
        Deltay: 2D numpy array, new coordinate vector in configuration space of 3 sections (y-axis).

    Returns:
        tube_para: Dictionary containing parameters for the tube plot.
        circle_handles: List of matplotlib plot handles for the circles.
    """
    # Basic settings
    n = 15  # Number of points for each circle
    # pressure = [0.1, 0.1, 1, 1, 0.1, 0.1]  # Pressure settings for color matrix
    colors = []  # Color matrix for plotting

    # Configuration parameters
    s1, deltax1, deltay1 = S, Deltax, Deltay
    # s2, deltax2, deltay2 = S[:, 1], Deltax[:, 1], Deltay[:, 1]
    # s3, deltax3, deltay3 = S[:, 2], Deltax[:, 2], Deltay[:, 2]

    # Diameter settings
    d = 0.02 # Diameter of the base [m]
    r = d / 2

    # Circle coordinates
    span_circle = np.arange(-np.pi, np.pi + np.pi / 20, 0.1)
    x_circ = r * np.cos(span_circle)
    y_circ = r * np.sin(span_circle)
    z_circ = np.zeros(len(x_circ))
    circle0 = np.stack((x_circ, y_circ, z_circ), axis=1)

    # PCC forward kinematics
    def compute_rotation_matrix(deltax, deltay, delta):
        return np.array([
            [1 + (deltax ** 2 / delta ** 2) * (np.cos(delta) - 1), (deltax * deltay / delta ** 2) * (np.cos(delta) - 1), (-deltax / delta) * np.sin(delta)],
            [(deltax * deltay / delta ** 2) * (np.cos(delta) - 1), 1 + (deltay ** 2 / delta ** 2) * (np.cos(delta) - 1), (-deltay / delta) * np.sin(delta)],
            [(deltax / delta) * np.sin(delta), (deltay / delta) * np.sin(delta), np.cos(delta)]
        ])

    delta1 = np.sqrt(deltax1 ** 2 + deltay1 ** 2)
    # delta2 = np.sqrt(deltax2 ** 2 + deltay2 ** 2)
    # delta3 = np.sqrt(deltax3 ** 2 + deltay3 ** 2)

    R1 = compute_rotation_matrix(deltax1, deltay1, delta1)
    # R12 = compute_rotation_matrix(deltax2, deltay2, delta2)
    # R2 = R1 @ R12
    # R23 = compute_rotation_matrix(deltax3, deltay3, delta3)
    # R3 = R2 @ R23

    # Simulating curve with segmented linear trajectory
    idx = np.linspace(0, 1, 20)

    section1, section2, section3 = [], [], []
    for m in idx:
        section1.append((s1 / delta1 ** 2) * np.array([
            deltax1 * (np.cos(m * delta1) - 1),
            deltay1 * (np.cos(m * delta1) - 1),
            delta1 * np.sin(m * delta1)
        ]))
        # colors.append(np.ones(n + 1) * pressure[0])

    section1 = np.array(section1)

    # for m in idx:
    #     section12 = (s2 / delta2 ** 2) * np.array([
    #         deltax2 * (np.cos(m * delta2) - 1),
    #         deltay2 * (np.cos(m * delta2) - 1),
    #         delta2 * np.sin(m * delta2)
    #     ])
    #     section2.append(section1[-1] + R1 @ section12)
    #     colors.append(np.ones(n + 1) * pressure[4])

    # section2 = np.array(section2)

    # for m in idx:
    #     section23 = (s3 / delta3 ** 2) * np.array([
    #         deltax3 * (np.cos(m * delta3) - 1),
    #         deltay3 * (np.cos(m * delta3) - 1),
    #         delta3 * np.sin(m * delta3)
    #     ])
    #     section3.append(section2[-1] + R2 @ section23)
    #     colors.append(np.ones(n + 1) * pressure[6])

    # section3 = np.array(section3)
    sections = np.vstack([section1]).T

    # Draw circles at the end of each segment
    circle1, circle2, circle3 = [], [], []
    for i in range(len(x_circ)):
        circle1.append(section1[-1] + R1 @ circle0[i])
        # circle2.append(section2[-1] + R2 @ circle0[i])
        # circle3.append(section3[-1] + R3 @ circle0[i])

    circle1 = np.array(circle1)
    # circle2 = np.array(circle2)
    # circle3 = np.array(circle3)

    # Tubeplot parameters
    # tube_para = {
    #     "sections": sections,
    #     "r": r,
    #     "colors": np.array(colors).T,
    #     "n": n,
    #     "tol": 0.0001
    # }

    # # Plot circles
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # ax.set_box_aspect([1, 1, 1])
    # # 手动设置坐标范围，确保各轴范围一致
    # ax.set_xlim([-1, 1])
    # ax.set_ylim([-1, 1])
    # ax.set_zlim([-1, 1])

    # circle_0 = ax.plot(circle0[:, 0], circle0[:, 1], circle0[:, 2], color='k', linewidth=1.5)
    # circle_1 = ax.plot(circle1[:, 0], circle1[:, 1], circle1[:, 2], color='k', linewidth=1.5)
    # circle_2 = ax.plot(circle2[:, 0], circle2[:, 1], circle2[:, 2], color='k', linewidth=1.5)
    # circle_3 = ax.plot(circle3[:, 0], circle3[:, 1], circle3[:, 2], color='k', linewidth=1.5)

    #plot curve
    # print(sections.shape)
    # print(sections)
    # section_x = sections[1,:]
    # section_y = sections[2,:]
    # section_z = sections[3,:]
    # print(section_x)
    # print(section_y)
    # print(section_z)
    # ax.plot(sections[0,:], sections[1,:], sections[2,:], color='blue', label="Path")

    circles = [circle0, circle1]

    return sections, circles
