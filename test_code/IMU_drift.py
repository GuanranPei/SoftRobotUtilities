import numpy as np
import robotic_rotation as rr
import tentacle_utility as tu
from visualization_pcc import plot_single_sections as plt_soft
import matplotlib.pyplot as plt
import plot_quaternion as plt_quat

if __name__ == "__main__":
    S = np.array([0.08, 0.08, 0.08])

    # setup serial port
    port_imu = "COM6"
    sensorobj = tu.setup_serial_port(port_imu)

    # Plot circles
    fig = plt.figure(1)
    ax = fig.add_subplot(111, projection='3d')

    fig2 = plt.figure(2)
    ax2 = fig2.add_subplot(111, projection='3d')

    while True:
        # Clear previous plots
        ax.cla()
        ax2.cla()
        sensor_val = tu.sensor_call(sensorobj)
        if sensor_val is not False:
            qbase = sensor_val[0]
            qtip = sensor_val[1]

            qbase_inv = rr.quatinv(qbase)

            qbase_1 = rr.quatmultiply(qbase_inv,qtip)

            eulbase_1 = rr.quat2eul(qbase_1)
            eulbase_1 = np.rad2deg(eulbase_1)

            plt_quat.plot_quaternion(qbase, ax=ax)
            plt.pause(0.01)
            plt_quat.plot_quaternion(qtip, ax=ax2)
            plt.pause(0.01)
            