import matplotlib.pyplot as plt
import serial
import matplotlib.animation as animation
import numpy as np
import time


fig, axs = plt.subplots(3)
fig.suptitle("Prediction")


ser=serial.Serial("COM5",115200)

y1_list=[]
y2_list=[]
y3_list=[]


def animate(i):
    global y1_list
    global y2_list
    global y3_list
    global y4_list
    data=ser.readline().decode('utf-8')
    package=data.split('/')
    x=float(package[0])
    y=float(package[1])
    theta=float(package[2])
    y1_list.append(x)
    y2_list.append(y)
    y3_list.append(theta)

    y1_list=y1_list[-100:]
    y2_list=y2_list[-100:]
    y3_list=y3_list[-100:]
   
    axs[0].clear()
    axs[1].clear()
    axs[0].plot(np.array(y1_list),'b')
    axs[1].plot(np.array(y2_list),'r')
    axs[0].set_xlim([0,100])
    axs[0].set_ylim([-50, 50])
    axs[1].set_xlim([0,100])
    axs[1].set_ylim([-50, 50])

    axs[2].clear()
    axs[2].plot(np.array(y3_list),'b')
    axs[2].set_xlim([0,100])
    axs[2].set_ylim([0, 65000])
    
   

ani = animation.FuncAnimation(fig, animate, frames=100, interval=10)

plt.show()
ser.close()


# while True:
#     data=str(ser.readline().decode('utf-8'))
#     package=data.split('/')
#     cleaned_data = [item.replace('\x00', '').strip() for item in package]
#     try:
#         M1_rpm=float(package[0])
#         M2_rpm=float(package[1])
#         x_prediction=float(package[2])
#         y_prediction=float(package[3])
#         theta_prediction=float(package[4])
#         print(str(M1_rpm)+' '+str(M2_rpm)+' '+str(x_prediction)+' '+str(y_prediction)+' '+str(theta_prediction))
#     except :
#         pass

