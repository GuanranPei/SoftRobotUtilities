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

# Dynamixel setting
DEVICENAME      = "COM4"
CURRENT_MODE    = 0 
EXPOSITION_MODE = 4

# Robot Setting
MOTOR_IDs                   = np.array([1, 2, 3, 4, 5, 6, 7, 8, 9])
Motor_Num                   = MOTOR_IDs.shape[1]
MOTOR_Orients               = np.array([1, 1, 1, 1, 1, 1, 1, 1, 1]) # motors 1,2,3,4,5,6,7,8,9 counter-clockwise rotating for compressing the soft section
TENSION_CURRENT_VALUE       = 15
GOAL_CURRENT       = TENSION_CURRENT_VALUE * MOTOR_Orients

motors_r                    = np.array([20000, 20000, 20000, 40000, 40000, 40000, 40000, 40000, 40000]).reshape(-1,1)