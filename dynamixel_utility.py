#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Author: Guanran Pei
# *********     Dynamixel utility library for soft robot control      *********

import os

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
ADDR_PRO_OPERATING_MODE     = 11               # address for Operating_Mode
ADDR_PRO_HOMING_OFFSET      = 20

# Data Byte Length
LEN_PRO_GOAL_CURRENT        = 2
LEN_PRO_PRESENT_CURRENT     = 2
LEN_PRO_GOAL_POSITION       = 4
LEN_PRO_PRESENT_POSITION    = 4

# Protocol version
PROTOCOL_VERSION            = 2.0              # See which protocol version is used in the Dynamixel

''' 
Operating_Mod
0	Current Control Mode	DYNAMIXEL only controls current(torque) regardless of speed and current. This mode is ideal for a gripper or a system that only uses current(torque) control or a system that has additional velocity/current controllers.
1	Velocity Control Mode	This mode controls velocity. This mode is identical to the Wheel Mode(endless) from existing DYNAMIXEL. This mode is ideal for wheel-type robots.
3(Default)	Position Control Mode	This mode controls current. This mode is identical to the Joint Mode from existing DYNAMIXEL. Operating current range is limited by the Max Position Limit(48) and the Min Position Limit(52). This mode is ideal for articulated robots that each joint rotates less than 360 degrees.
4	Extended Position Control Mode(Multi-turn)	This mode controls current. This mode is identical to the Multi-turn Position Control from existing DYNAMIXEL. 512 turns are supported(-256[rev] ~ 256[rev]). This mode is ideal for multi-turn wrists or conveyer systems or a system that requires an additional reduction gear. Note that Max Position Limit(48), Min Position Limit(52) are not used on Extended Position Control Mode.
5	Current-based Position Control Mode	This mode controls both current and current(torque). Up to 512 turns are supported(-256[rev] ~ 256[rev]). This mode is ideal for a system that requires both current and current control such as articulated robots or grippers.
16	PWM Control Mode (Voltage Control Mode)	This mode directly controls PWM output. (Voltage Control Mode)
'''

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torquef

def porthandler(DEVICENAME):
    portHandler = PortHandler(DEVICENAME)
    return portHandler

def packethandler():
    packetHandler = PacketHandler(PROTOCOL_VERSION)
    return packetHandler
     
def openPort(portHandler):
    if portHandler.openPort():
        print("Succeeded to open the port")
        return True
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()
        return False
    
def setBaudrate(portHandler, BAUDRATE):
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
        return True
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()
        return False
    
def setOperatingMode(portHandler, packetHandler, MOTOR_IDs, Operating_Mode):
    Motor_Num = len(MOTOR_IDs)
    for i in range(Motor_Num):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, MOTOR_IDs[i], ADDR_PRO_OPERATING_MODE, Operating_Mode)
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Set Motor {MOTOR_IDs[i]} Operating Mode Error: {packetHandler.getTxRxResult(dxl_comm_result)}")
            return False
        elif dxl_error != 0:
            print(f"Set Motor {MOTOR_IDs[i]} Operating Mode Error: {packetHandler.getRxPacketError(dxl_error)}")
            return False
        else:
            print(f"Operating Mode set to Mode {Operating_Mode} for ID {MOTOR_IDs[i]}")
            return True

def setTorque(portHandler, packetHandler, MOTOR_IDs, TORQUE_STATUS):
    Motor_Num = len(MOTOR_IDs)
    for i in range(Motor_Num):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, MOTOR_IDs[i], ADDR_PRO_TORQUE_ENABLE, TORQUE_STATUS)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            return False
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
            return False
        else:
            print("Dynamixel#%d has been successfully connected" % MOTOR_IDs[i])
            return True

def groupWriteCurrentNum(portHandler, packetHandler):
    groupwrite_current_num = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_CURRENT, LEN_PRO_GOAL_CURRENT)
    return groupwrite_current_num

def groupReadCurrentNum(portHandler, packetHandler):
    groupread_current_num = GroupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_CURRENT, LEN_PRO_PRESENT_CURRENT)
    return groupread_current_num

def groupWritePositionNum(portHandler, packetHandler):
    groupwrite_position_num = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION)
    return groupwrite_position_num

def groupReadPositionNum(portHandler, packetHandler):
    groupread_position_num = GroupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
    return groupread_position_num

def groupWriteSync(packetHandler, groupwrite_num,  MOTOR_IDs, goal_values, datatype):
    Motor_Num = len(MOTOR_IDs)
    for i in range(Motor_Num):
        if datatype == 'current':
            # Allocate goal position value into byte array
            goal_current = [DXL_LOBYTE(DXL_LOWORD(goal_values[i])), DXL_HIBYTE(DXL_LOWORD(goal_values[i])), DXL_LOBYTE(DXL_HIWORD(goal_values[i])), DXL_HIBYTE(DXL_HIWORD(goal_values[i]))]
            # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
            dxl_addparam_result = groupwrite_num.addParam(MOTOR_IDs[i], goal_current)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % MOTOR_IDs[i])
                quit()
                return False
        elif datatype == 'position':
            # Allocate goal position value into byte array
            goal_position = [DXL_LOBYTE(DXL_LOWORD(goal_values[i])), DXL_HIBYTE(DXL_LOWORD(goal_values[i])), DXL_LOBYTE(DXL_HIWORD(goal_values[i])), DXL_HIBYTE(DXL_HIWORD(goal_values[i]))]
            # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
            dxl_addparam_result = groupwrite_num.addParam(MOTOR_IDs[i], goal_position)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % MOTOR_IDs[i])
                quit()
                return False
        else:
            print("There is no such datatype!")
            return False

    # Syncwrite goal position
    dxl_comm_result = groupwrite_num.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        # Clear syncwrite parameter storage
        groupwrite_num.clearParam()
        return False
    else:
        groupwrite_num.clearParam()
        return True

def groupReadSync(packetHandler, groupread_num,  MOTOR_IDs, datatype):
    # Add parameter storage for Dynamixel#i present position value
    Motor_Num = len(MOTOR_IDs)
    for i in range(Motor_Num):
        dxl_addparam_result = groupread_num.addParam(MOTOR_IDs[i])
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncRead addparam failed" % MOTOR_IDs[i])
            quit()
            return False

    # Syncread present position
    dxl_comm_result = groupread_num.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        return False
      
    dxl_present = [0] * len(MOTOR_IDs)
    Motor_Num = len(MOTOR_IDs)
    for i in range(Motor_Num):
        if datatype == 'current':
            # Check if groupsyncread data of Dynamixel#i is available
            dxl_getdata_result = groupread_num.isAvailable(MOTOR_IDs[i], ADDR_PRO_PRESENT_CURRENT, LEN_PRO_PRESENT_CURRENT)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncRead getdata failed" % MOTOR_IDs[i])
                quit()
                return False
            # Get Dynamixel#1 present current value
            dxl_present[i] = groupread_num.getData(MOTOR_IDs[i], ADDR_PRO_PRESENT_CURRENT, LEN_PRO_PRESENT_CURRENT)
        elif datatype == 'position':
            # Check if groupsyncread data of Dynamixel#i is available
            dxl_getdata_result = groupread_num.isAvailable(MOTOR_IDs[i], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncRead getdata failed" % MOTOR_IDs[i])
                quit()
                return False
            # Get Dynamixel#1 present position value
            dxl_present[i] = groupread_num.getData(MOTOR_IDs[i], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
        else:
            print("There is no such datatype!")
            return False
    
    # Clear syncread parameter storage
    groupread_num.clearParam()

    return dxl_present

def ReadHomingOffset(portHandler, packetHandler,  MOTOR_IDs):

    homing_offsets = []  # To store raw homing offset values
    homing_offsets_dec = []  # To store signed homing offset values

    Motor_Num = len(MOTOR_IDs)
    for i in range(Motor_Num):
        # Read 4-byte homing offset
        dxl_homing_offset, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, MOTOR_IDs[i], ADDR_PRO_HOMING_OFFSET)
        # Check for communication errors
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Communication Error for motor ID {MOTOR_IDs[i]}: {packet_handler.getTxRxResult(dxl_comm_result)}")
            return False
        elif dxl_error != 0:
            print(f"Hardware Error for motor ID {MOTOR_IDs[i]}: {packet_handler.getRxPacketError(dxl_error)}")
            return False

        # Append the raw offset value
        homing_offsets.append(dxl_homing_offset)
        
        # Convert to signed 32-bit integer if needed
        if dxl_homing_offset > 0x7FFFFFFF:  # Check if MSB (sign bit) is 1
            dxl_homing_offset_signed = dxl_homing_offset - 0x100000000  # Convert to signed
        else:
            dxl_homing_offset_signed = dxl_homing_offset
        
        # Append the signed offset value
        homing_offsets_dec.append(dxl_homing_offset_signed)

    return homing_offsets_dec

def WriteHomingOffset(portHandler, packetHandler,  MOTOR_IDs, HOMING_OFFSETS_dec):

    # Convert signed integers to unsigned 32-bit integers
    homing_offsets = [offset & 0xFFFFFFFF for offset in HOMING_OFFSETS_dec]

    success_count = 0

    Motor_Num = len(MOTOR_IDs)
    for i in range(Motor_Num):
        # Disable torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, MOTOR_IDs[i], ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Communication Error for motor ID {MOTOR_IDs[i]}: {packetHandler.getTxRxResult(dxl_comm_result)}")
            return False
        elif dxl_error != 0:
            print(f"Hardware Error for motor ID {MOTOR_IDs[i]}: {packetHandler.getRxPacketError(dxl_error)}")
            return False

        # Write homing offset
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, MOTOR_IDs[i], ADDR_PRO_HOMING_OFFSET, homing_offsets[i])
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Communication Error for motor ID {MOTOR_IDs[i]}: {packetHandler.getTxRxResult(dxl_comm_result)}")
            return False
        elif dxl_error != 0:
            print(f"Hardware Error for motor ID {MOTOR_IDs[i]}: {packetHandler.getRxPacketError(dxl_error)}")
            return False

        # Success message
        print(f"Writing homing offset for motor ID {MOTOR_IDs[i]} successfully.") 

    return True

def ShutDown(portHandler, packetHandler, MOTOR_IDs):

    Motor_Num = len(MOTOR_IDs)
    for i in range(Motor_Num):
        # Disable Dynamixel Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, MOTOR_IDs[i], ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            return False
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
            return False

    # Close port
    portHandler.closePort()
    return True