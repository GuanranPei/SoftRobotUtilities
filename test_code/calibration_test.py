#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from tkinter import ttk
import time
import os

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
MOTOR_IDs                   = np.array([1, 2, 3, 4, 5, 6, 7, 8, 9])
Motor_Num                   = MOTOR_IDs.shape[0]
MOTOR_Orients               = np.array([1, 1, 1, 1, 1, 1, 1, 1, 1]) # motors 1,2,3,4,5,6,7,8,9 counter-clockwise rotating for compressing the soft section
CURRENT_THRESHOLD           = 1
POSITION_THRESHOLD          = 10
TENSION_CURRENT_VALUE       = 10
GOAL_CURRENT       = TENSION_CURRENT_VALUE * MOTOR_Orients

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

