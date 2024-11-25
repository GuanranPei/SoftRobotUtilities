#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from PyQt5 import QtWidgets

import sys
sys.path.append("../")

import dynamixel_utility as du
import robotic_rotation as rr
import softarm_utility as su
import kinematics_l2delta as kl2d
import kinematics_delta2x as kd2x

# ======================================================================================
#                                   Dynamixel Setting
# ======================================================================================
DEVICENAME      = "COM5"
BAUDRATE        = 1000000
CURRENT_MODE    = 0 
EXPOSITION_MODE = 4

# ======================================================================================
#                                   Robot Setting
# ======================================================================================
MOTOR_R                     = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0])
MOTOR_IDs                   = np.array([1, 2, 3, 4, 5, 6, 7, 8, 9])
Motor_Num                   = MOTOR_IDs.shape[0]
MOTOR_Orients               = np.array([1, 1, 1, 1, 1, 1, 1, 1, 1]) # motors 1,2,3,4,5,6,7,8,9 counter-clockwise rotating for compressing the soft section
CURRENT_THRESHOLD           = 1
POSITION_THRESHOLD          = 10
TENSION_CURRENT_VALUE       = 10
MOTOR_TENSION_CURRENT       = TENSION_CURRENT_VALUE * MOTOR_Orients

# ======================================================================================
#                                   IMUs Setup
# ======================================================================================
# setup serial port
port_imu = "COM14"
sensorobj = su.setup_serial_port(port_imu)

# For IMU sensing
Timu_1 = rr.eul2rotm(np.deg2rad([60,0,0]))
Timu_2 = rr.eul2rotm(np.deg2rad([30,0,0]))
Timu_3 = rr.eul2rotm(np.deg2rad([0,0,0]))

# ======================================================================================
#                                   Dynamixel Setup
# ======================================================================================
# Initialize PortHandler Structs
port_handler = du.porthandler(DEVICENAME)

# Initialize PacketHandler Structs
packet_handler = du.packethandler()

# Open port
output_openPort = du.openPort(port_handler)

# Set port baudrate
output_setBaudRate = du.setBaudrate(port_handler, BAUDRATE)

# Set current control Operating_Mode for all motors
output_setOperatingMode = du.setOperatingMode(port_handler, packet_handler, MOTOR_IDs, CURRENT_MODE)

# Enable Dynamixel Torque for all motors
output_setTorque = du.setTorque(port_handler, packet_handler, MOTOR_IDs, 1)

# Initialize Groupsyncwrite Current Structs
groupwrite_current_num = du.groupWriteCurrentNum(port_handler, packet_handler)
# Initialize groupReadSync Current Structs
groupread_current_num = du.groupReadCurrentNum(port_handler, packet_handler)
# Initialize Groupsyncwrite Position Structs
groupwrite_position_num = du.groupWritePositionNum(port_handler, packet_handler)
# Initialize groupReadSync Position Structs
groupread_position_num = du.groupReadPositionNum(port_handler, packet_handler)

# Write the goal current value to motors
output_groupWriteSync = du.groupWriteSync(packet_handler, groupwrite_current_num,  MOTOR_IDs, MOTOR_TENSION_CURRENT, 'current')
while 1:
    MOTOR_CURRENT = du.groupReadSync(packet_handler, groupread_current_num, MOTOR_IDs, 'current')
    if np.all(np.abs(MOTOR_CURRENT - MOTOR_TENSION_CURRENT) <= CURRENT_THRESHOLD):
        break

# ======================================================================================
#                                   Qt Plotting Setup
# ======================================================================================
# Generate data storage for the plotting
maxDataPoints = 50
time_cal = np.arange(1, maxDataPoints + 1)
storage_base_1 = np.zeros((maxDataPoints, 3))
storage_1_2 = np.zeros((maxDataPoints, 3))
storage_2_3 = np.zeros((maxDataPoints, 3))

# Generate the Qt application
app = QtWidgets.QApplication([])

# ======================================================================================
#                                   1st Section Calibration
# ======================================================================================

# Generate plot window
win = pg.GraphicsLayoutWidget(show=True)
win.setWindowTitle("Real-Time IMU base 1")

# Generate plot area
plot = win.addPlot(title="IMU base 1")  # 设置标题
plot.setLabel('bottom', 'newest points')  # 设置 x 轴标签
plot.setLabel('left', 'angle/deg')  # 设置 y 轴标签
plot.showGrid(x=True, y=True)  # 显示网格
plot.addLegend()  # 添加图例
plot.setXRange(np.min(time_cal), np.max(time_cal))  # 设置 x 轴范围

# Plot the initial curve
pitch_base_1 = plot.plot(time_cal, storage_base_1[:, 1], pen=pg.mkPen(color='r', width=2), name="pitch")  # 红色线
roll_base_1 = plot.plot(time_cal, storage_base_1[:, 2], pen=pg.mkPen(color='g', width=2), name="roll")  # 绿色线

# Add the reference dash line
yline = pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen(color='k', style=pg.QtCore.Qt.DashLine, width=2))
plot.addItem(yline)

# Data updating function
def update1():
    global storage_base_1
    while 1:
        sensor_val = su.sensor_call(sensorobj)
        if sensor_val is not False:
            qbase = sensor_val[0]
            q1 = sensor_val[1]
            qbase_inv = rr.quatinv(qbase)
            RIMUbase_1 = rr.quat2rotm(rr.quatmultiply(qbase_inv, q1))
            RROBObase_1 = np.transpose(Timu_1) @ RIMUbase_1 @ Timu_1
            eulbase_1 = np.rad2deg(rr.rotm2eul(RROBObase_1))
            break
    storage_base_1 = np.vstack((storage_base_1[1:], np.array(eulbase_1).reshape(1,-1)))
    pitch_base_1.setData(time_cal, storage_base_1[:,1])
    roll_base_1.setData(time_cal, storage_base_1[:,2])

# Create time counter for update the plot
timer = QtCore.QTimer()
timer.timeout.connect(update1)
timer.start(20) # update per 20 ms

# Homingoffset calculating and saving function
def stop_plot1():
    timer.stop()  # 停止定时器
    win.close()  # 关闭窗口
    button.close()
    main_window.close()

# generate stop button
button = QtWidgets.QPushButton("Save_Cal")  # 使用 QtWidgets.QPushButton
button.clicked.connect(stop_plot1)  # 按钮点击后触发关闭功能

# link button and plot
layout = QtWidgets.QVBoxLayout()  # 使用 QtWidgets.QVBoxLayout
layout.addWidget(win)  # 添加绘图窗口
layout.addWidget(button)  # 添加按钮

# generate main_win to contain plot_win and button
main_window = QtWidgets.QWidget()  # 使用 QtWidgets.QWidget
main_window.setLayout(layout)
main_window.resize(800, 600)
main_window.setWindowTitle("Real-Time IMU base 1")
main_window.show()

# start the plot cycle
app.exec_()

# Homingoffset calculating and saving
print("Calibration for 1st section is done!")
# read the current homing offset
homingoffsets1_c = du.ReadHomingOffset(port_handler, packet_handler,  MOTOR_IDs[:3])
# read the current motor position
motor_position1 = du.groupReadSync(packet_handler, groupread_position_num,  MOTOR_IDs[:3], 'position')
# calculate proper homing offset for the 1st section
homingoffsets1_d = MOTOR_R[:3] - motor_position1 + homingoffsets1_c
# write the proper homing offset
output_WriteHomingOffset = du.WriteHomingOffset(port_handler, packet_handler, MOTOR_IDs[:3], homingoffsets1_d)
# change the Operating_Mode to extended position control
output_setOperatingMode = du.setOperatingMode(port_handler, packet_handler, MOTOR_IDs[:3], EXPOSITION_MODE)
# Enable Dynamixel Torque for motors 1, 2, 3
output_setTorque = du.setTorque(port_handler, packet_handler, MOTOR_IDs[:3], 1)
# write the motors 1 2 3 to reference position
output_groupWriteSync = du.groupWriteSync(packet_handler, groupwrite_position_num,  MOTOR_IDs[:3], MOTOR_R[:3], 'position')
while 1:
    motor_position1 = du.groupReadSync(packet_handler, groupread_position_num, MOTOR_IDs[:3], 'position')
    if np.all(np.abs(motor_position1 - MOTOR_R[:3]) <= POSITION_THRESHOLD):
        break

# ======================================================================================
#                                   2nd Section Calibration
# ======================================================================================

# Generate plot window
win = pg.GraphicsLayoutWidget(show=True)
win.setWindowTitle("Real-Time IMU 1 2")

# Generate plot area
plot = win.addPlot(title="IMU 1 2")  # 设置标题
plot.setLabel('bottom', 'newest points')  # 设置 x 轴标签
plot.setLabel('left', 'angle/deg')  # 设置 y 轴标签
plot.showGrid(x=True, y=True)  # 显示网格
plot.addLegend()  # 添加图例
plot.setXRange(np.min(time_cal), np.max(time_cal))  # 设置 x 轴范围

# Plot the initial curve
pitch_1_2 = plot.plot(time_cal, storage_1_2[:, 1], pen=pg.mkPen(color='r', width=2), name="pitch")  # 红色线
roll_1_2 = plot.plot(time_cal, storage_1_2[:, 2], pen=pg.mkPen(color='g', width=2), name="roll")  # 绿色线

# Add the reference dash line
yline = pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen(color='k', style=pg.QtCore.Qt.DashLine, width=2))
plot.addItem(yline)

# Data updating function
def update2():
    global storage_1_2
    while 1:
        sensor_val = su.sensor_call(sensorobj)
        if sensor_val is not False:
            q1 = sensor_val[1]
            q2 = sensor_val[2]
            q1_inv = rr.quatinv(q1)
            RIMU1_2 = rr.quat2rotm(rr.quatmultiply(q1_inv, q2))
            RROBO1_2 = np.transpose(Timu_2) @ RIMU1_2 @ Timu_2
            eul1_2 = np.rad2deg(rr.rotm2eul(RROBO1_2))
            break
    storage_1_2 = np.vstack((storage_1_2[1:], np.array(eul1_2).reshape(1,-1)))
    pitch_1_2.setData(time_cal, storage_1_2[:,1])
    roll_1_2.setData(time_cal, storage_1_2[:,2])

# Create time counter for update the plot
timer = QtCore.QTimer()
timer.timeout.connect(update2)
timer.start(20) # update per 20 ms

# Homingoffset calculating and saving function
def stop_plot2():
    timer.stop()  # 停止定时器
    win.close()  # 关闭窗口
    button.close()
    main_window.close()

# generate stop button
button = QtWidgets.QPushButton("Save_Cal")  # 使用 QtWidgets.QPushButton
button.clicked.connect(stop_plot2)  # 按钮点击后触发关闭功能

# link button and plot
layout = QtWidgets.QVBoxLayout()  # 使用 QtWidgets.QVBoxLayout
layout.addWidget(win)  # 添加绘图窗口
layout.addWidget(button)  # 添加按钮

# generate main_win to contain plot_win and button
main_window = QtWidgets.QWidget()  # 使用 QtWidgets.QWidget
main_window.setLayout(layout)
main_window.resize(800, 600)
main_window.setWindowTitle("Real-Time IMU 1 2")
main_window.show()

# start the plot cycle
app.exec_()

# Homingoffset calculating and saving
print("Calibration for 2nd section is done!")
# read the current homing offset
homingoffsets2_c = du.ReadHomingOffset(port_handler, packet_handler,  MOTOR_IDs[3:6])
# read the current motor position
motor_position2 = du.groupReadSync(packet_handler, groupread_position_num,  MOTOR_IDs[3:6], 'position')
# calculate proper homing offset for the 2nd section
homingoffsets2_d = MOTOR_R[3:6] - motor_position2 + homingoffsets2_c
# write the proper homing offset
output_WriteHomingOffset = du.WriteHomingOffset(port_handler, packet_handler, MOTOR_IDs[3:6], homingoffsets2_d)
# change the Operating_Mode to extended position control
output_setOperatingMode = du.setOperatingMode(port_handler, packet_handler, MOTOR_IDs[3:6], EXPOSITION_MODE)
# Enable Dynamixel Torque for motors 4, 5, 6
output_setTorque = du.setTorque(port_handler, packet_handler, MOTOR_IDs[3:6], 1)
# write the motors 4 5 6 to reference position
output_groupWriteSync = du.groupWriteSync(packet_handler, groupwrite_position_num,  MOTOR_IDs[3:6], MOTOR_R[3:6], 'position')
while 1:
    motor_position2 = du.groupReadSync(packet_handler, groupread_position_num, MOTOR_IDs[3:6], 'position')
    if np.all(np.abs(motor_position2 - MOTOR_R[3:6]) <= POSITION_THRESHOLD):
        break

# ======================================================================================
#                                   3rd Section Calibration
# ======================================================================================

# Generate plot window
win = pg.GraphicsLayoutWidget(show=True)
win.setWindowTitle("Real-Time IMU 2 3")

# Generate plot area
plot = win.addPlot(title="IMU 2 3")  # 设置标题
plot.setLabel('bottom', 'newest points')  # 设置 x 轴标签
plot.setLabel('left', 'angle/deg')  # 设置 y 轴标签
plot.showGrid(x=True, y=True)  # 显示网格
plot.addLegend()  # 添加图例
plot.setXRange(np.min(time_cal), np.max(time_cal))  # 设置 x 轴范围

# Plot the initial curve
pitch_2_3 = plot.plot(time_cal, storage_2_3[:, 1], pen=pg.mkPen(color='r', width=2), name="pitch")  # 红色线
roll_2_3 = plot.plot(time_cal, storage_2_3[:, 2], pen=pg.mkPen(color='g', width=2), name="roll")  # 绿色线

# Add the reference dash line
yline = pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen(color='k', style=pg.QtCore.Qt.DashLine, width=2))
plot.addItem(yline)

# Data updating function
def update3():
    global storage_2_3
    while 1:
        sensor_val = su.sensor_call(sensorobj)
        if sensor_val is not False:
            q2 = sensor_val[2]
            q3 = sensor_val[3]
            q2_inv = rr.quatinv(q2)
            RIMU2_3 = rr.quat2rotm(rr.quatmultiply(q2_inv, q3))
            RROBO2_3 = np.transpose(Timu_3) @ RIMU2_3 @ Timu_3
            eul2_3 = np.rad2deg(rr.rotm2eul(RROBO2_3))
            break
    storage_2_3 = np.vstack((storage_2_3[1:], np.array(eul2_3).reshape(1,-1)))
    pitch_2_3.setData(time_cal, storage_2_3[:,1])
    roll_2_3.setData(time_cal, storage_2_3[:,2])

# Create time counter for update the plot
timer = QtCore.QTimer()
timer.timeout.connect(update3)
timer.start(20) # update per 20 ms

# Homingoffset calculating and saving function
def stop_plot3():
    timer.stop()  # 停止定时器
    win.close()  # 关闭窗口
    button.close()
    main_window.close()

# generate stop button
button = QtWidgets.QPushButton("Save_Cal")  # 使用 QtWidgets.QPushButton
button.clicked.connect(stop_plot3)  # 按钮点击后触发关闭功能

# link button and plot
layout = QtWidgets.QVBoxLayout()  # 使用 QtWidgets.QVBoxLayout
layout.addWidget(win)  # 添加绘图窗口
layout.addWidget(button)  # 添加按钮

# generate main_win to contain plot_win and button
main_window = QtWidgets.QWidget()  # 使用 QtWidgets.QWidget
main_window.setLayout(layout)
main_window.resize(800, 600)
main_window.setWindowTitle("Real-Time IMU 2 3")
main_window.show()

# start the plot cycle
app.exec_()

# Homingoffset calculating and saving
print("Calibration for 3rd section is done!")
# read the current homing offset
homingoffsets3_c = du.ReadHomingOffset(port_handler, packet_handler,  MOTOR_IDs[6:9])
# read the current motor position
motor_position3 = du.groupReadSync(packet_handler, groupread_position_num,  MOTOR_IDs[6:9], 'position')
# calculate proper homing offset for the 3rd section
homingoffsets3_d = MOTOR_R[6:9] - motor_position3 + homingoffsets3_c
# write the proper homing offset
output_WriteHomingOffset = du.WriteHomingOffset(port_handler, packet_handler, MOTOR_IDs[6:9], homingoffsets3_d)
# change the Operating_Mode to extended position control
output_setOperatingMode = du.setOperatingMode(port_handler, packet_handler, MOTOR_IDs[6:9], EXPOSITION_MODE)
# Enable Dynamixel Torque for motors 7, 8, 9
output_setTorque = du.setTorque(port_handler, packet_handler, MOTOR_IDs[6:9], 1)
# write the motors 7 8 9 to reference position
output_groupWriteSync = du.groupWriteSync(packet_handler, groupwrite_position_num,  MOTOR_IDs[6:9], MOTOR_R[6:9], 'position')
while 1:
    motor_position3 = du.groupReadSync(packet_handler, groupread_position_num, MOTOR_IDs[6:9], 'position')
    if np.all(np.abs(motor_position3 - MOTOR_R[6:9]) <= POSITION_THRESHOLD):
        break

# ======================================================================================
#                                   Tendon Lengths Calibration
# ======================================================================================

# 获取用户输入
zpos_base = float(input("Input base position [mm]: "))
zpos_1 = float(input("Input 1st tip position [mm]: "))
zpos_2 = float(input("Input 2nd tip position [mm]: "))
zpos_3 = float(input("Input 3rd tip position [mm]: "))

# 计算参考腱长度
len_1 = 0.001 * (zpos_base - 1 - zpos_1)
len_2 = 0.001 * (zpos_1 - zpos_2)
len_3 = 0.001 * (zpos_2 - zpos_3)

# 创建 L_R 数组
L_R = np.array([len_1, len_1, len_1, len_2, len_2, len_2, len_3, len_3, len_3])

# 保存为 .npy 文件（推荐用于高效数据存储）
np.save("L_R.npy", L_R)
print("Data saved to L_R.npy")

# ======================================================================================
#                                   Motor Position Validate
# ======================================================================================
MOTOR_POSITION = du.groupReadSync(packet_handler, groupread_position_num, MOTOR_IDs, 'position')
print(MOTOR_POSITION)

# ======================================================================================
#                                   Close everything
# ======================================================================================
output_ShutDown = du.ShutDown(port_handler, packet_handler, MOTOR_IDs)