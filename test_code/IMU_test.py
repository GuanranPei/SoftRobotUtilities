import numpy as np
import time
import sys

sys.path.append("../")

import softarm_utility as su
import robotic_rotation as rr

if __name__ == "__main__":
    # Initialize system parameters
    motors_r = np.array([20000, 20000, 20000, 40000, 40000, 40000, 40000, 40000, 40000]).reshape(-1, 1)
    L_r = np.array([0.1320, 0.1320, 0.1320, 0.2740, 0.2740, 0.2740, 0.2680, 0.2680, 0.2680]).reshape(-1, 1)
    unit_scale = 2 * np.pi / 4096
    r_pulley = 0.020 / 2
    d = 0.054 / 2

    # setup serial port
    port_imu = "COM3"
    sensorobj = su.setup_serial_port(port_imu)

    # For IMU sensing
    Timu_1 = rr.eul2rotm(np.deg2rad([60,0,0]))
    Timu_2 = rr.eul2rotm(np.deg2rad([30,0,0]))
    Timu_3 = rr.eul2rotm(np.deg2rad([0,0,0]))

    while True:
        sensor_val = su.sensor_call(sensorobj)

        if sensor_val is not False:
            qbase = sensor_val[0]
            q1 = sensor_val[1]
            q2 = sensor_val[2]
            q3 = sensor_val[3]

            eulbase = su.quat2eul(qbase)
            eul1 = su.quat2eul(q1)
            eul2 = su.quat2eul(q2)
            eul3 = su.quat2eul(q3)

            # print(f"eul_base(ZYX) \t{eulbase[0]}, {eulbase[1]}, {eulbase[2]}")
            # print(f"eul_1(ZYX) \t\t{eul1[0]}, {eul1[1]}, {eul1[2]}")
            # print(f"eul_2(ZYX) \t\t{eul2[0]}, {eul2[1]}, {eul2[2]}")
            # print(f"eul_3(ZYX) \t\t{eul3[0]}, {eul3[1]}, {eul3[2]}")

            qbase_inv = su.quatinv(qbase)
            q1_inv = su.quatinv(q1)
            q2_inv = su.quatinv(q2)

            RIMUbase_1 = su.quat2rotm(su.quatmultiply(qbase_inv, q1))
            RIMU1_2 = su.quat2rotm(su.quatmultiply(q1_inv, q2))
            RIMU2_3 = su.quat2rotm(su.quatmultiply(q2_inv, q3))

            RROBObase_1 = np.transpose(Timu_1) @ RIMUbase_1 @ Timu_1
            RROBO1_2 = np.transpose(Timu_2) @ RIMU1_2 @ Timu_2
            RROBO2_3 = np.transpose(Timu_3) @ RIMU2_3 @ Timu_3

            eulbase_1 = su.rotm2eul(RROBObase_1)
            eul1_2 = su.rotm2eul(RROBO1_2)
            eul2_3 = su.rotm2eul(RROBO2_3)

            print(f"eul_base_1(ZYX) \t{eulbase_1[0]}, {eulbase_1[1]}, {eulbase_1[2]}")
            print(f"eul_1_2(ZYX) \t\t{eul1_2[0]}, {eul1_2[1]}, {eul1_2[2]}")
            print(f"eul_2_3(ZYX) \t\t{eul2_3[0]}, {eul2_3[1]}, {eul2_3[2]}")

            # Placeholder for pose_recon_R function, should be implemented accordingly
            # sdxdy_c = np.vstack([su.pose_recon_R(L[0:3, :], RROBObase_1),
            #                      su.pose_recon_R(L[3:6, :], RROBO1_2),
            #                      su.pose_recon_R(L[6:9, :], RROBO2_3)])

            # Print formatted table (commented out for brevity)
            # print('+------------+------------+------------+------------+------------+------------+------------+------------+------------+')
            # print('| {:10} | {:10} | {:10} | {:10} | {:10} | {:10} | {:10} | {:10} | {:10} |'.format('s1', 'deltax1', 'deltay1', 's2', 'deltax2', 'deltay2', 's3', 'deltax3', 'deltay3'))
            # print('+------------+------------+------------+------------+------------+------------+------------+------------+------------+')
            # print('| {:10.4f} | {:10.4f} | {:10.4f} | {:10.4f} | {:10.4f} | {:10.4f} | {:10.4f} | {:10.4f} | {:10.4f} |'.format(*sdxdy_c[:, 0]))
            # print('+------------+------------+------------+------------+------------+------------+------------+------------+------------+')

            time.sleep(0.1)