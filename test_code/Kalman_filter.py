import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# 读取 CSV 文件，假设文件名为 "data.csv"
df = pd.read_csv("IMU_sampling_processed.csv", header=[0, 1, 2])

# 提取 base1, base2, base3 的四元数
base1_quaternions = df["base1"]["rotation"][["qw", "qx", "qy", "qz"]].values
base2_quaternions = df["base2"]["rotation"][["qw", "qx", "qy", "qz"]].values
base3_quaternions = df["base3"]["rotation"][["qw", "qx", "qy", "qz"]].values

# 转换为 NumPy 矩阵
IMU_base1 = np.array(base1_quaternions)
IMU_base2 = np.array(base2_quaternions)
IMU_base3 = np.array(base3_quaternions)

# 读取 CSV 文件，假设文件名为 "data.csv"
df = pd.read_csv("MC_sampling_processed.csv", header=[0, 1, 2])

# 提取 base1, base2, base3 的四元数
base1_quaternions = df["base1"]["rotation"][["qw", "qx", "qy", "qz"]].values
base2_quaternions = df["base2"]["rotation"][["qw", "qx", "qy", "qz"]].values
base3_quaternions = df["base3"]["rotation"][["qw", "qx", "qy", "qz"]].values

# 转换为 NumPy 矩阵
MC_base1 = np.array(base1_quaternions)
MC_base2 = np.array(base2_quaternions)
MC_base3 = np.array(base3_quaternions)

# 读取 CSV 文件，假设文件名为 "data.csv"
df = pd.read_csv("Tendons_sampling_processed.csv", header=[0, 1, 2])

# 提取 base1, base2, base3 的四元数
base1_quaternions = df["base1"]["rotation"][["qw", "qx", "qy", "qz"]].values
base2_quaternions = df["base2"]["rotation"][["qw", "qx", "qy", "qz"]].values
base3_quaternions = df["base3"]["rotation"][["qw", "qx", "qy", "qz"]].values

# 转换为 NumPy 矩阵
Tendons_base1 = np.array(base1_quaternions)
Tendons_base2 = np.array(base2_quaternions)
Tendons_base3 = np.array(base3_quaternions)

# 初始化变量
x = np.array([0.512932,0.000255,-0.858429,-0.000272])  # 初始状态（四元数）
P = np.eye(4) * 0.01        # 初始协方差
Q = np.zeros((4,4))        # 过程噪声协方差
Q[0,0] = 0.1
Q[1,1] = 0.0001
Q[2,2] = 0.1
Q[3,3] = 0.1
R_imu = np.zeros((4,4))    # IMU 测量噪声协方差
R_imu[0,0] = 0.0001
R_imu[1,1] = 0.1
R_imu[2,2] = 0.0001
R_imu[3,3] = 0.0001
H = np.eye(4)               # 观测矩阵

error_tendons = []
error_imu = []
error_fusion = []

for i in range(1000):
    # prediction phase
    x_pred = Tendons_base3[i,:]
    P_pred = Q

    z_imu = IMU_base3[i,:]

    K = P_pred @ H.T @ np.linalg.inv(H @ P_pred @ H.T + R_imu)
    x = x_pred + K @ (z_imu - H @ x_pred)
    P = (np.eye(4)-K@H)@P_pred

    # 四元数归一化
    x = x / np.linalg.norm(x)
    # print(x)
    z_mc = MC_base3[i,:]

    error_tendons.append(np.sum(z_mc-x_pred))
    error_imu.append(np.sum(z_mc-z_imu))
    error_fusion.append(np.sum(z_mc-x))

# 创建 figure
fig = plt.figure()  # 创建一个 8x6 英寸的图形窗口
ax = fig.add_subplot(1, 1, 1)  # 添加一个子图 (1 行, 1 列, 第 1 个)
ax.plot(range(1000), error_tendons, label="tendons", color="red")
ax.plot(range(1000), error_imu, label="imu", color="green")
ax.plot(range(1000), error_fusion, label="fusion", color="blue")

# 添加图例
ax.legend()

# 显示图形
plt.show()

# print(z_mc)

# print(np.sum(z_mc-x))
# print(np.sum(z_mc-np.array([0.512932,0.000255,-0.858429,-0.000272])))
# print(np.sum(z_mc-z_imu))