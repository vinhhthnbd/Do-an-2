import numpy as np
import serial
from rplidar import RPLidar
import ransac
import ekfslam
import threading
import time

# initialize the robot
robot = ekfslam.EKFSLAM(max_landmark=20, x_error=0.02, y_error=0.02, theta_error=0.01, range_error=0.1, bearing_error=np.deg2rad(2), threshold=5.991)
robot.mean[0][0] = 0
robot.mean[1][0] = 0
robot.mean[2][0] = 0
mcu = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
time.sleep(1)

lidar = RPLidar('/dev/ttyUSB0')

info = lidar.get_info()
print(info)

health = lidar.get_health()
print(health)

# MCU's variables
sl = 0
sr = 0

# observation matrix
z = 0

# finished observing flag
flag = 0

def slam():
    global sl, sr, flag

    while True:
        receive = mcu.readline().decode("utf-8")
        receive_list = receive.split("/")
        sl, sr = [float(val) for val in receive_list]
        delta_theta = (sr - sl)/(2 * 74)
        s = (sr + sl)/2
        delta_x = s * np.cos(robot.mean[2][0] + delta_theta/2)
        delta_y = s * np.sin(robot.mean[2][0] + delta_theta/2)
        robot.predict_location(delta_x, delta_y, delta_theta)
        
        if flag > 3:
            z_t = z.copy()
            ret1, ret2, ret3, obs = ransac.ransacAlgorithm(z_t, 20, 5, np.deg2rad(10), 10, 40)

            robot.correct_location(obs)
            command = f'{robot.mean[0][0]:.4f} {robot.mean[1][0]:.4f} {robot.mean[2][0]:.4f}'
            send = f'{command:<64}'
            mcu.write(send.encode("utf-8"))
            print(obs.shape)
            flag = 0

slam_thread = threading.Thread(target=slam)
slam_thread.daemon = True
slam_thread.start()

# main thread
try:
    for scan in lidar.iter_scans():
        x_list = []
        y_list = []
        r_list = []
        phi_list = []

        if sl == 0 and sr == 0:
            # read lidar's data
            for meas in scan:
                rad = np.deg2rad(-meas[1]) # convert degree to radian
                phi = np.arctan2(np.sin(rad), np.cos(rad)) # values between -pi to pi
                r = meas[2] # unit mm
                x_list.append(robot.mean[0][0] + r*np.cos(robot.mean[2][0] + phi))
                y_list.append(robot.mean[1][0] + r*np.sin(robot.mean[2][0] + phi))
                r_list.append(r)
                phi_list.append(phi)

            # lidar's observation matrix
            z = np.array([x_list, y_list, r_list, phi_list])
            
            # set finished flag to 1
            flag += 1

        else:
            flag = 0

except KeyboardInterrupt:
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    print(robot.mean)
    print(robot.covariance)
    print(robot.Nt)
