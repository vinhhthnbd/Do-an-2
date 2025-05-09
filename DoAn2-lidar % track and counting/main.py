import numpy as np
import serial
from rplidar import RPLidar
import ransac
import extended_kalman_filter as ekf
import threading

robot = ekf.EKFSLAM(10, 0.1, 0.01, np.pi/180)
lidar = RPLidar('COM10')
mcu = serial.Serial('COM0')
z = 0
flag = 0

def lidar_scans():
    global z, flag
    for scan in lidar.iter_scans():
        x_list = []
        y_list = []
        r_list = []
        phi_list = []

        try:
            mcu.write("e".encode("utf-8"))
            data = mcu.readline().decode("utf-8")
            data_list = data.split("_")
            robot.delta_x, robot.delta_y, robot.delta_theta = [float(val) for val in data_list]
        except:
            pass

        for meas in scan:
            rad = np.deg2rad(-meas[1]) # convert degree to radian
            phi = np.arctan2(np.sin(rad), np.cos(rad)) # values between -pi to pi
            r = meas[2] # unit mm
            x_list.append(robot.mean[0][0] + robot.delta_x + r*np.cos(robot.delta_theta + robot.mean[2][0] + phi))
            y_list.append(robot.mean[1][0] + robot.delta_y + r*np.sin(robot.delta_theta + robot.mean[2][0] + phi))
            r_list.append(r)
            phi_list.append(phi)

        z = np.array([x_list, y_list, r_list, phi_list])
        flag = 1

    lidar.stop()
    lidar.disconnect()

lidar_thread = threading.Thread(target=lidar_scans)
lidar_thread.daemon = True 
lidar_thread.start()

# remove bluetooth
while True:
    if flag == 1:
        ret1, ret2, obs = ransac.ransac_algorithm(z, 20, 5, np.deg2rad(5), 10, 20)
        robot.prediction()
        robot.correction(obs)
        mcu.write('R{fx}/{fy}/{ftheta}\n'.format(fx=robot.mean[0][0],fy=robot.mean[1][0],ftheta=robot.mean[2][0]).encode('utf-8'))
        flag = 0