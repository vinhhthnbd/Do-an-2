import numpy as np
from rplidar import RPLidar
import threading
from ransac import *
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

fig = plt.figure()
ax = fig.add_subplot(111)
lidar = RPLidar('COM10')
iterator = lidar.iter_scans()
z = 0
flag = 0

def lidar_scans():
    global z, flag
    for scan in lidar.iter_scans():
        x_list = []
        y_list = []
        r_list = []
        phi_list = []

        for meas in scan:
            rad = np.deg2rad(-meas[1]) # convert degree to radian
            phi = np.arctan2(np.sin(rad), np.cos(rad)) # values between -pi to pi
            r = meas[2] # unit mm
            x_list.append(r*np.cos(phi))
            y_list.append(r*np.sin(phi))
            r_list.append(r)
            phi_list.append(phi)

        z = np.array([x_list, y_list, r_list, phi_list])
        flag = 1

def animate(i):
    global z

    ret1, ret2, ret3, obs = ransacAlgorithm(z.copy(), 10, 10, np.deg2rad(10), 20, 45)

    ax.clear()
    ax.plot(z[0,:],z[1,:], '.b')
    ax.plot(obs[0,:],obs[1,:],'.r')
    ax.set_ylim([-6000, 6000])
    ax.set_xlim([-6000,6000])
    xn = np.arange(-10000,10000,5000)
    for i in range(ret1.shape[0]):
        yn = np.polyval([-ret1[i]/ret2[i],-ret3[i]/ret2[i]],xn)
        ax.plot(xn,yn)

    print("Number of landmarks: {}".format(obs.shape[1]))

lidar_thread = threading.Thread(target=lidar_scans)
lidar_thread.start()

time.sleep(2)

ani = animation.FuncAnimation(fig, animate, frames=100, interval=100)

plt.show()
lidar.stop()
lidar.disconnect()