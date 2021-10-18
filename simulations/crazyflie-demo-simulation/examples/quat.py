import numpy as np
import matplotlib.pyplot as plt
import os

# get the necessary states
my_data = np.load('../files/logs/save-flight-07.15.2021_10.43.03.npy')
states = my_data['states']
allstates = my_data['allstates']
time = my_data['timestamps']
# allstates[0,3,155:] *= -1
# allstates[0,6,155:] *= -1
plt.plot(time[0,:], allstates[0,1,:], time[0,:], allstates[0,2,:], time[0,:], allstates[0,6,:],
         time[0,:], allstates[0,3,:], time[0,:], allstates[0,7,:])
plt.legend(['y', 'z', 'q0', 'q1', 'roll (Euler)'])
plt.show()

RPM = allstates[0, 16:20, :]
# compute the torque and force inputs from the RPMs, using the control allocation matrix
l = 0.034  # L/sqrt(2) if L is the length of the arm of the drone
b = 7.6406e-11  # 7.94e-12  # drag constant, called km in cf2x.urdf
k = 1.7899e-8  # 3.16e-10  # thrust constant, called kf in cf2x.urdf
A = np.array([[1, 1, 1, 1],[-l, -l, l, l],[l, -l, -l, l], [b/k, -b/k, b/k, -b/k]])
            # control allocation matrix, see 5.5 in Crazyflie modelling and identification by AIMotionLab
om = 2*np.pi/60*RPM
input = np.dot(A, k*om**2)

time = time[0, :]
y = allstates[0, 1, :]
z = allstates[0, 2, :]
q0 = allstates[0, 6, :]
q1 = allstates[0, 3, :]
# with open(os.path.dirname(os.path.abspath(__file__)) + "/../files/logs/trajs.npy", 'wb') as out_file:
#     np.savez(out_file, time=time, y=y, z=z, q0=q0, q1=q1, input=input)
