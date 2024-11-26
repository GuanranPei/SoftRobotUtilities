import numpy as np
import serial
import serial.tools.list_ports
import re


def setup_serial_port(port_name):
    ports = serial.tools.list_ports.comports()
    if ports:
        for port in ports:
            try:
                ser = serial.Serial(port.device)
                ser.close()
                del ser
            except Exception as e:
                print(f"Error closing port {port.device}: {e}")
        print('All serial ports are closed.')
    else:
        print('There is no opened object.')
    arduinoObj = serial.Serial(port_name, 115200, timeout=1)
    arduinoObj.write_terminator = '\r\n'
    arduinoObj.read_terminator = '\r\n'
    arduinoObj.reset_input_buffer()
    arduinoObj.reset_output_buffer()
    print('Serialport is configured')
    return arduinoObj

def sensor_call(SerialObj):
    SerialObj.reset_input_buffer()
    try:
        sensor_string = SerialObj.readline().decode('utf-8', errors='ignore').strip()
        float_pattern = r'[-+]?\d*\.\d+|\d+'
        data = re.findall(float_pattern, sensor_string)
        # print(data)
        if len(data) == 8 and all(len(d) >= 6 for d in data):
            sensor_val = np.array([
                [float(data[0]), float(data[1]), float(data[2]), float(data[3])],
                [float(data[4]), float(data[5]), float(data[6]), float(data[7])]
            ])
        else:
            sensor_val = False
    except Exception as e:
        print(f"Error reading from serial port: {e}")
        sensor_val = False
    return sensor_val

def pcc_recon_R(L, R):
    theta = np.arccos(R[2, 2])
    if np.sin(theta) == 0:
        deltax = 0
        deltay = 0
        print("WARNING: the denominator is zero, please check the input if!")
    else:
        deltax = 0.5 * (R[2, 0] - R[0, 2]) * theta / np.sin(theta)
        deltay = 0.5 * (R[2, 1] - R[1, 2]) * theta / np.sin(theta)
    s = np.mean(L)
    sdxdy = np.array([s, deltax, deltay])
    return sdxdy