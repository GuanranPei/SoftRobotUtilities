import sys
sys.path.append("../")

import numpy as np
import robotic_rotation as rr
import tentacle_utility as tu
from test_code.visualization_pcc import plot_single_sections as plt_soft
import matplotlib.pyplot as plt

if __name__ == "__main__":
    S = np.array([0.08, 0.08, 0.08])

    # setup serial port
    port_imu = "COM8"
    sensorobj = tu.setup_serial_port(port_imu)

    # Plot circles
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # ax.set_box_aspect([1, 1, 1])
    # # 手动设置坐标范围，确保各轴范围一致
    # ax.set_xlim([-0.1, 0.1])
    # ax.set_ylim([-0.1, 0.1])
    # ax.set_zlim([-0.1, 0.1])
    # ax.view_init(elev=0, azim=0)

    while True:
        # # 清除旧的绘图
        # ax.cla()
        # ax.set_box_aspect([1, 1, 1])
        # # 手动设置坐标范围，确保各轴范围一致
        # ax.set_xlim([-0.1, 0.1])
        # ax.set_ylim([-0.1, 0.1])
        # ax.set_zlim([-0.1, 0.1])
        # ax.view_init(elev=0, azim=0)

        sensor_val = tu.sensor_call(sensorobj)
        if sensor_val is not False:
            qbase = sensor_val[0]
            qtip = sensor_val[1]
            eulbase = rr.quat2eul(qbase)
            eulbase = np.rad2deg(eulbase)
            eultip = rr.quat2eul(qtip)
            eultip = np.rad2deg(eultip)
            # print(f"eul_base(ZYX) \t{eulbase[0]}, {eulbase[1]}, {eulbase[2]}")
            # print(f"eul_tip(ZYX) \t{eultip[0]}, {eultip[1]}, {eultip[2]}")

            qbase_inv = rr.quatinv(qbase)

            qbase_1 = rr.quatmultiply(qbase_inv,qtip)

            eulbase_1 = rr.quat2eul(qbase_1)
            eulbase_1 = np.rad2deg(eulbase_1)
            print(f"eul_base_1(ZYX) \t{eulbase_1}")

            Rbase_1 = rr.quat2rotm(qbase_1)
            sdxdy = tu.pcc_recon_R(S,Rbase_1)
            # print(f"sdxdy \t{sdxdy}")

            # sections, [circle0, circle1] = plt_soft(sdxdy[0], sdxdy[1], sdxdy[2])

            # circle_0 = ax.plot(circle0[:, 0], circle0[:, 1], circle0[:, 2], color='k', linewidth=1.5)
            # circle_1 = ax.plot(circle1[:, 0], circle1[:, 1], circle1[:, 2], color='k', linewidth=1.5)

            # # Plot central axis
            # ax.plot(sections[0,:], sections[1,:], sections[2,:], color='blue', label="Path")

            # plt.pause(0.001)
