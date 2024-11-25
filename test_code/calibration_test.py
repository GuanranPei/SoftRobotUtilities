#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui

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
DEVICENAME      = "COM4"
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
# Initialize Groupsyncread Current Structs
groupread_current_num = du.groupReadCurrentNum(port_handler, packet_handler)
# Initialize Groupsyncwrite Position Structs
groupwrite_position_num = du.groupWritePositionNum(port_handler, packet_handler)
# Initialize Groupsyncread Position Structs
groupread_position_num = du.groupReadPositionNum(port_handler, packet_handler)

# Write the goal current value to motors
output_groupWriteSync = du.groupWriteSync(packet_handler, groupwrite_current_num,  MOTOR_IDs, MOTOR_TENSION_CURRENT, 'current')
while 1:
    MOTOR_CURRENT = du.GroupSyncRead(packet_handler, groupread_current_num, MOTOR_IDs, 'current')
    if np.all(np.abs(MOTOR_CURRENT - MOTOR_TENSION_CURRENT) <= CURRENT_THRESHOLD):
        break

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
#                                   Qt Plotting Setup
# ======================================================================================
# Generate data storage for the plotting
maxDataPoints = 50
time_cal = np.arange(1, maxDataPoints + 1)
storage_base_1 = np.zeros((maxDataPoints, 3))
storage_1_2 = np.zeros((maxDataPoints, 3))
storage_2_3 = np.zeros((maxDataPoints, 3))

# Generate the Qt application
app = QtGui.QApplication([])

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
def update():
    sensor_val = su.sensor_call(sensorobj)
    if sensor_val is not False:
        qbase = sensor_val[0]
        q1 = sensor_val[1]
        qbase_inv = rr.quatinv(qbase)
        RIMUbase_1 = rr.quat2rotm(rr.quatmultiply(qbase_inv, q1))
        RROBObase_1 = np.transpose(Timu_1) @ RIMUbase_1 @ Timu_1
        eulbase_1 = np.rad2deg(rr.rotm2eul(RROBObase_1))
        storage_base_1 = np.vstack((storage_base_1[1:], eulbase_1.reshape(1,-1)))
        pitch_base_1.setData(time_cal, storage_base_1[:,1])
        roll_base_1.setData(time_cal, storage_base_1[:,2])

# Create time counter for update the plot
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(20) # update per 20 ms

# Homingoffset calculating and saving function
def stop_plot():
    timer.stop()  # 停止定时器
    win.close()  # 关闭窗口
    button.close()
    main_window.close()

    # Homingoffset calculating and saving
    print("Calibration for 1st section is done!")
    # read the current homing offset
    homingoffsets1_c = du.ReadHomingOffset(port_handler, packet_handler,  MOTOR_IDs[:3])
    # read the current motor position
    motor_position1 = du.GroupSyncRead(packet_handler, groupread_position_num,  MOTOR_IDs[:3], 'position')
    # calculate proper homing offset for the 1st section
    homingoffsets1_d = MOTOR_R[:3] - motor_position1 + homingoffsets1_c
    # write the proper homing offset
    output_WriteHomingOffset = du.WriteHomingOffset(port_handler, packet_handler, MOTOR_IDs[:3], homingoffsets1_d)
    # change the Operating_Mode to extended position control
    output_setOperatingMode = du.setOperatingMode(port_handler, packet_handler, MOTOR_IDs, EXPOSITION_MODE)
    # Enable Dynamixel Torque for motors 1, 2, 3
    output_setTorque = du.setTorque(port_handler, packet_handler, MOTOR_IDs[:3], 1)
    # write the motors 1 2 3 to reference position
    output_groupWriteSync = du.groupWriteSync(packet_handler, groupwrite_position_num,  MOTOR_IDs[:3], MOTOR_R[:3], 'position')
    while 1:
        motor_position1 = du.GroupSyncRead(packet_handler, groupread_position_num, MOTOR_IDs[:3], 'position')
        if np.all(np.abs(motor_position1 - MOTOR_R[:3]) <= POSITION_THRESHOLD):
            break

# generate stop button
button = QtGui.QPushButton("Save_Cal")
button.clicked.connect(stop_plot)  # 按钮点击后触发关闭功能

# link button and plot
layout = QtGui.QVBoxLayout()  # 创建一个垂直布局
layout.addWidget(win)  # 添加绘图窗口
layout.addWidget(button)  # 添加按钮

# generate main_win to contain plot_win and button
main_window = QtGui.QWidget()
main_window.setLayout(layout)
main_window.resize(800, 600)
main_window.setWindowTitle("Real-Time IMU base 1")
main_window.show()

# start the plot cycle
app.exec_()