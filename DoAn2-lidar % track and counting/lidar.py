import numpy as np
from rplidar import RPLidar
import serial
import threading

lidar = RPLidar('COM10')
ser = serial.Serial('/dev/tty') 

x = 0
y = 0
theta = 0
delta_x = 0
delta_y = 0
delta_theta = 0

def lidar_scans(lidar, ser):
    global x, y, theta, delta_x, delta_y, delta_theta, z, flag
    for scan in lidar.iter_scans():
        x_list = []
        y_list = []
        r_list = []
        phi_list = []

        try:
            ser.write("e".encode("utf-8"))
            data = ser.readline().decode("utf-8")
            data_list = data.split("_")
            delta_x, delta_y, delta_theta = [float(val) for val in data_list]
        except:
            pass

        for meas in scan:
            rad = np.deg2rad(-meas[1]) # convert degree to radian
            phi = np.arctan2(np.sin(rad), np.cos(rad)) # values between -pi to pi
            r = meas[2] # unit mm
            x_list.append(x + delta_x + r*np.cos(delta_theta + theta + phi))
            y_list.append(y + delta_y + r*np.sin(delta_theta + theta + phi))
            r_list.append(r)
            phi_list.append(phi)

        z = np.array([x_list, y_list, r_list, phi_list])
        flag = 1

    lidar.stop()
    lidar.disconnect()

lidar_scans(lidar, ser)