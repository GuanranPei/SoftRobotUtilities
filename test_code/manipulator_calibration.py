#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from tkinter import ttk
import time
import os

import softarm_utility as su

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

# Control table address
ADDR_PRO_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_POSITION   = 132
ADDR_PRO_GOAL_CURRENT       = 102
ADDR_PRO_PRESENT_CURRENT    = 126
ADDR_PRO_Operating_Mode     = 11               # address for Operating_Mode, change it to current control model
ADDR_PRO_HOMING_OFFSET      = 20

# Data Byte Length
LEN_PRO_GOAL_CURRENT        = 2
LEN_PRO_PRESENT_CURRENT     = 2
LEN_PRO_GOAL_POSITION       = 4
LEN_PRO_PRESENT_POSITION    = 4

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Operating_Mod
Operating_Mode              = 0                 # 0	Current Control Mode; 1	Velocity Control Mode; 3(Default) Position Control Mode; 4 Extended Position Control Mode(Multi-turn); 5 Current-based Position Control Mode

# Default setting
BAUDRATE                    = 1000000           # Dynamixel default baudrate : 1000000
DEVICENAME                  = 'COM4'            # port name of u2d2 control board

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 1                 # Dynamixel moving status threshold

# customized setting
MOTOR_IDs                   = np.array([1, 2, 3, 4, 5, 6, 7, 8, 9])
Motor_Num                   = MOTOR_IDs.shape[1]
MOTOR_Orients               = np.array([1, 1, 1, 1, 1, 1, 1, 1, 1]) # motors 1,2,3,4,5,6,7,8,9 counter-clockwise rotating for compressing the soft section
TENSION_CURRENT_VALUE       = 15
GOAL_CURRENT       = TENSION_CURRENT_VALUE * MOTOR_Orients

motors_r                    = np.array([20000, 20000, 20000, 40000, 40000, 40000, 40000, 40000, 40000]).reshape(-1,1)

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupSyncWrite instance for Present Current
groupSyncWrite_current = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_CURRENT, LEN_PRO_GOAL_CURRENT)

# Initialize GroupSyncRead instace for Present Current
groupSyncRead_current = GroupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_CURRENT, LEN_PRO_PRESENT_CURRENT)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

# Enable Dynamixel Torque and set Operating_Mode for all motors
for motor_id in MOTOR_IDs:
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % motor_id)

# Add Dynamixel present current values of all motors to the SyncRead storage
for motor_id in MOTOR_IDs:
    dxl_addparam_result = groupSyncRead_current.addParam(motor_id)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncRead_current addparam failed" % motor_id)
        quit()

# Add Dynamixel goal current values of all motors to the Syncwrite storage
for motor_id, goal_current in zip(MOTOR_IDs, GOAL_CURRENT):
    # need to check
    para_goal_current = goal_current.astype(np.int16)
    para_goal_current = para_goal_current.astype(np.uint16)
    dxl_addparam_result = groupSyncWrite_current.addParam(motor_id, int(para_goal_current))
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite_current addparam failed" % motor_id)
        quit()

# Syncwrite goal position
dxl_comm_result = groupSyncWrite_current.txPacket()
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

# Clear syncwrite parameter storage
groupSyncWrite_current.clearParam()

# define variable to store present current
PRESENT_CURRENT = np.zeros((9,1))
while 1:
    # Syncread present current
    dxl_comm_result = groupSyncRead_current.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Check if groupSyncRead_current data of Dynamixel#1 is available
    for motor_id in MOTOR_IDs:
        dxl_getdata_result = groupSyncRead_current.isAvailable(motor_id, ADDR_PRO_PRESENT_CURRENT, LEN_PRO_PRESENT_CURRENT)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead_current getdata failed" % motor_id)
            quit()

    # Get Dynamixel present current value
    for index, motor_id in enumerate(MOTOR_IDs):
        PRESENT_CURRENT[index,:] = groupSyncRead_current.getData(motor_id, ADDR_PRO_PRESENT_CURRENT, LEN_PRO_PRESENT_CURRENT)
    PRESENT_CURRENT.astype(np.uint16)
    PRESENT_CURRENT.astype(np.int16)
    
    flag = 0
    for goal_current, present_current in zip(GOAL_CURRENT, PRESENT_CURRENT):
        if not (abs(goal_current - present_current) > DXL_MOVING_STATUS_THRESHOLD):
            pass
        else:
            flag += 1
    
    if flag == 0:
        break

# Calibration GUI
# initialize the storage variable
maxDataPoints = 50
time_data = np.arange(1, maxDataPoints + 1)
storage_base_1 = np.zeros((maxDataPoints, 3))
storage_1_2 = np.zeros((maxDataPoints, 3))
storage_2_3 = np.zeros((maxDataPoints, 3))

# generate gui button
root = tk.Tk()
root.title('IMU Orientation Graphic')

# generate matplotlib figure
fig, axs = plt.subplots(3, 1, figsize=(10, 8))

# generate calibration datastreaming for the first section
pitch_base_1, = axs[0].plot(time_data, storage_base_1[:, 1], 'r', linewidth=2, label='pitch')
roll_base_1, = axs[0].plot(time_data, storage_base_1[:, 2], 'g', linewidth=2, label='roll')
axs[0].axhline(0, color='k', linestyle='--', linewidth=2, label='y = 0')
axs[0].set_title('base_1')
axs[0].set_xlabel('newest points')
axs[0].set_ylabel('angle/deg')
axs[0].legend()
axs[0].set_xlim([min(time_data), max(time_data)])

# generate calibration datastreaming for the second section
pitch_1_2, = axs[1].plot(time_data, storage_1_2[:, 1], 'r', linewidth=2, label='pitch')
roll_1_2, = axs[1].plot(time_data, storage_1_2[:, 2], 'g', linewidth=2, label='roll')
axs[1].axhline(0, color='k', linestyle='--', linewidth=2, label='y = 0')
axs[1].set_title('1_2')
axs[1].set_xlabel('newest points')
axs[1].set_ylabel('angle/deg')
axs[1].legend()
axs[1].set_xlim([min(time_data), max(time_data)])

# generate calibration datastreaming for the third section
pitch_2_3, = axs[2].plot(time_data, storage_2_3[:, 1], 'r', linewidth=2, label='pitch')
roll_2_3, = axs[2].plot(time_data, storage_2_3[:, 2], 'g', linewidth=2, label='roll')
axs[2].axhline(0, color='k', linestyle='--', linewidth=2, label='y = 0')
axs[2].set_title('2_3')
axs[2].set_xlabel('newest points')
axs[2].set_ylabel('angle/deg')
axs[2].legend()
axs[2].set_xlim([min(time_data), max(time_data)])

# manage the layout
plt.tight_layout()

# embedd Matplotlib figure in the Tkinter window
canvas = FigureCanvasTkAgg(fig, master=root)
canvas.draw()
canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

# callback function for the button
def on_button_click():
    global running
    print("Save Cal button clicked")
    running = False

# generate button and link it to the Tkinter
button = ttk.Button(root, text='Save Cal', command=on_button_click)
button.pack()

# open IMU serial port
port_imu = "COM9"
sensorobj = su.setup_serial_port(port_name = port_imu)

# rotation matrix transformation of IMU
Timu_1 = su.rotz(60)
Timu_2 = su.rotz(30)
Timu_3 = su.rotz(0)

# read the sensor data and update it
def update_data():
    global storage_base_1, storage_1_2, storage_2_3

    sensor_val = su.sensor_call(sensorobj)
    if sensor_val:
        qbase = sensor_val[0]
        q1 = sensor_val[1]
        q2 = sensor_val[2]
        q3 = sensor_val[3]

        qbase_inv = su.quatinv(qbase)
        q1_inv = su.quatinv(q1)
        q2_inv = su.quatinv(q2)

        RIMUbase_1 = su.quat2rotm(su.quatmultiply(qbase_inv, q1))
        RIMU1_2 = su.quat2rotm(su.quatmultiply(q1_inv, q2))
        RIMU2_3 = su.quat2rotm(su.quatmultiply(q2_inv, q3))

        RROBObase_1 = np.transpose(Timu_1) @ RIMUbase_1 @ Timu_1
        RROBO1_2 = np.transpose(Timu_2) @ RIMU1_2 @ Timu_2
        RROBO2_3 = np.transpose(Timu_3) @ RIMU2_3 @ Timu_3

        eulbase_1 = np.rad2deg(su.quat2eul(RROBObase_1))
        eul1_2 = np.rad2deg(su.quat2eul(RROBO1_2))
        eul2_3 = np.rad2deg(su.quat2eul(RROBO2_3))

        storage_base_1 = np.vstack((storage_base_1[1:], eulbase_1))
        storage_1_2 = np.vstack((storage_1_2[1:], eul1_2))
        storage_2_3 = np.vstack((storage_2_3[1:], eul2_3))

        pitch_base_1.set_ydata(storage_base_1[:, 1])
        roll_base_1.set_ydata(storage_base_1[:, 2])
        pitch_1_2.set_ydata(storage_1_2[:, 1])
        roll_1_2.set_ydata(storage_1_2[:, 2])
        pitch_2_3.set_ydata(storage_2_3[:, 1])
        roll_2_3.set_ydata(storage_2_3[:, 2])

        canvas.draw()

# run the Tkinter mainloop and update loop
running = True

def run():
    while running:
        update_data()
        root.update_idletasks()
        root.update()
        time.sleep(0.01)  # 调整延迟以控制更新频率

    root.quit()  # 退出主循环

root.after(100, run)  # 延迟 100 毫秒启动 run 函数
root.mainloop()

# Write the calibration data in the motor
# Read the current homing-offset in the motors
HOMING_OFFSETS = np.zeros((Motor_Num,-1), dtype=np.uint32)
HOMING_OFFSETS_dec = np.zeros((Motor_Num,-1), dtype=np.int32)

for index, motor_id in enumerate(MOTOR_IDs):
    HOMING_OFFSETS[index], dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, motor_id, ADDR_PRO_HOMING_OFFSET)
    HOMING_OFFSETS[index] = HOMING_OFFSETS[index].astype(np.uint32)
    HOMING_OFFSETS_dec[index] = HOMING_OFFSETS[index].astype(np.int32)

    if dxl_comm_result != COMM_SUCCESS:
        print('%s' % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print('%s' % packetHandler.getRxPacketError(dxl_error))

# Read the present motor position
# Initialize GroupSyncRead instace for Present Position
groupSyncRead_position = GroupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

# Add Dynamixel present current values of all motors to the SyncRead storage
for motor_id in MOTOR_IDs:
    dxl_addparam_result = groupSyncRead_position.addParam(motor_id)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncRead_position addparam failed" % motor_id)
        quit()

# Syncread present position
dxl_comm_result = groupSyncRead_position.txRxPacket()
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

# Check if groupSyncRead_current data of Dynamixel#1 is available
for motor_id in MOTOR_IDs:
    dxl_getdata_result = groupSyncRead_position.isAvailable(motor_id, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
    if dxl_getdata_result != True:
        print("[ID:%03d] groupSyncRead_current getdata failed" % motor_id)
        quit()

# define variable to store present current
PRESENT_POSITION = np.zeros((9,1))

# Get Dynamixel present position value
for index, motor_id in enumerate(MOTOR_IDs):
    PRESENT_POSITION[index,:] = groupSyncRead_position.getData(motor_id, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
PRESENT_POSITION.astype(np.uint16)
PRESENT_POSITION.astype(np.int16)

# set proper homing offset
HOMING_OFFSETS_dec = motors_r - PRESENT_POSITION + HOMING_OFFSETS_dec
HOMING_OFFSETS = np.array([np.uint32(np.int32(h)) for h in HOMING_OFFSETS_dec]).reshape(-1,1)

for index, motor_id in enumerate(MOTOR_IDs):
    packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    packetHandler.write4ByteTxRx(portHandler, motor_id, ADDR_PRO_HOMING_OFFSET, HOMING_OFFSETS[index])
    dxl_comm_result = packetHandler.getLastTxRxResult(portHandler, COMM_SUCCESS)
    dxl_error = packetHandler.getLastRxPacketError(portHandler)
    if dxl_comm_result != COMM_SUCCESS:
        print('%s' % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print('%s' % packetHandler.getRxPacketError(dxl_error))
    else:
        print('Writing #%d homing offset successfully' % motor_id)

# after set proper homing offset, check if the present position is reasonable
# Syncread present position
dxl_comm_result = groupSyncRead_position.txRxPacket()
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

# Check if groupSyncRead_current data of Dynamixel is available
for motor_id in MOTOR_IDs:
    dxl_getdata_result = groupSyncRead_position.isAvailable(motor_id, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
    if dxl_getdata_result != True:
        print("[ID:%03d] groupSyncRead_current getdata failed" % motor_id)
        quit()

# Get Dynamixel present position value
for index, motor_id in enumerate(MOTOR_IDs):
    PRESENT_POSITION[index,:] = groupSyncRead_position.getData(motor_id, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
PRESENT_POSITION.astype(np.uint16)
PRESENT_POSITION.astype(np.int16)

# Close everything
for motor_id in MOTOR_IDs:
    packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    dxl_comm_result = packetHandler.getLastTxRxResult(portHandler, COMM_SUCCESS)
    dxl_error = packetHandler.getLastRxPacketError(portHandler)
    if dxl_comm_result != COMM_SUCCESS:
        print('%s' % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print('%s' % packetHandler.getRxPacketError(dxl_error))

# Close port
portHandler.closePort()