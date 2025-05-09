import numpy as np
from rplidar import RPLidar
import ransac
import matplotlib.pyplot as plt
lidar = RPLidar('COM10')

info = lidar.get_info()
print(info)

health = lidar.get_health()
print(health)

for i, scan in enumerate(lidar.iter_scans()):
    print('%d: Got %d measurments' % (i, len(scan)))
    if i > 20:
        break

lidar.stop()
lidar.stop_motor()
lidar.disconnect()

print(scan)

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

print(z)

ret1, ret2, obs = ransac.ransac_algorithm(z, 20, 5, np.deg2rad(5), 10, 20)

print(obs)

plt.plot(z[0,:],z[1,:], '.b')
plt.plot(obs[0,:],obs[1,:],'.r')

xn = np.arange(-10000,10000,500)
for i in range(ret1.shape[0]):
    yn = np.polyval([ret1[i],ret2[i]],xn)
    plt.plot(xn,yn)
plt.show()