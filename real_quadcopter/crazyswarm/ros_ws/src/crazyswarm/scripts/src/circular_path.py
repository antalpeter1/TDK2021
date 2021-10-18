import numpy as np
from matplotlib import pyplot as plt
import math

plt.close('all')
n_points = 100
acceleration_rounds = 0.5
acceleration_multiplier = 2
acceleration_multiplier = 0.2
i_step = 0.1
radious = 0.6


# FI
phi0 = 0
v0 = 0


poly7_x = []
poly7_y = []
    

phi = []
phi += [0]
# for i in range(0, n_points):
i = 0
while phi[-1] < math.pi:
    if len(phi) > 1:
        v = (phi[-1] - phi[-2]) / i_step
    else:
        v = v0
    phi += [1/2 * acceleration_multiplier * i_step**2+ v * i_step + phi[-1]]
    i += i_step
    t1 = i
        
 
phi = np.array(phi)
# compressing into a range of "acceleration_rounds", that is 2 * 2pi
phi = phi / phi[-1] * (math.pi) 

x = np.cos(phi.tolist()) * radious
y = np.sin(phi.tolist()) * radious
plt.plot(x, y)
plt.plot(x, y, 'k.')



t = np.linspace(0, t1 , len(x))
poly7_x = [np.poly1d(np.polyfit(t, x, deg=7))]
poly7_y = [np.poly1d(np.polyfit(t, y, deg=7))]


# poly7_x_t = [poly7_x(t_) for t_ in t]
# poly7_y_t = [poly7_y(t_) for t_ in t]

# plt.plot(poly7_x_t, poly7_y_t)


# plt.figure()
# plt.plot(x)
# plt.plot(y)


# 222222222
s0 = phi[-1]
v0 = (phi[-1] - phi[-2]) / i_step
a0 = (phi[-1] - phi[-2]) / i_step**2

phi = []
phi += [s0]
# for i in range(0, n_points):
i = i_step
while phi[-1] < 2 * math.pi:
    if len(phi) > 1:
        v = (phi[-1] - phi[-2]) / i_step
    else:
        v = v0
    phi += [1/2 * acceleration_multiplier * i_step**2+ v * i_step + phi[-1]]
    i += i_step
    t2 = i
    
phi = np.array(phi)
# compressing into a range of "acceleration_rounds", that is 2 * 2pi
phi = phi - [phi[0]]
phi = phi / phi[-1] * (math.pi) 
phi = phi + math.pi

# plt.figure()
x = np.cos(phi.tolist()) * radious
y = np.sin(phi.tolist()) * radious
plt.plot(x, y)
plt.plot(x, y, 'k.')


t = np.linspace(0, t2, len(x))
poly7_x += [np.poly1d(np.polyfit(t, x, deg=7))]
poly7_y += [np.poly1d(np.polyfit(t, y, deg=7))]



    
    
# 33333333 Now do a whole circle with the same speed
s0 = phi[-1]
v0 = (phi[-1] - phi[-2]) / i_step
a0 = (phi[-1] - phi[-2]) / i_step**2
    
phi = []
phi += [0]
i = i_step
while phi[-1] < 2 * math.pi:
    v = v0
    phi += [v * i_step + phi[-1]]
    i += i_step
    t3 = i

phi = np.array(phi)
# Let's compress it to 0 - 2pi
phi = phi / phi[-1] * (2*math.pi)
x = np.cos(phi.tolist()) * radious
y = np.sin(phi.tolist()) * radious
plt.plot(x, y)
# plt.plot(x, y, 'k.')


t = np.linspace(0, t3, len(x))
poly7_x += [np.poly1d(np.polyfit(t, x, deg=7))]
poly7_y += [np.poly1d(np.polyfit(t, y, deg=7))]


# Evaluating the polinome
# plt.figure()
# for i in range(2, len(poly7_x)):
#     # t = np.linspace(0.01, 1-0.01, 100)
#     t = np.linspace(0, 1, 100)
#     poly7_x_t = [poly7_x[i](t_) for t_ in t]
#     poly7_y_t = [poly7_y[i](t_) for t_ in t]
#     plt.plot(poly7_x_t, poly7_y_t)
#     # plt.plot(poly7_x_t, poly7_y_t, 'k.')

import os
import csv
cwd = os.getcwd()
T = [t1, t2, t3]


plt.figure()
i = 2
t = np.linspace(0, T[i], 100)
poly7_x_t = [poly7_x[i](t_) for t_ in t]
poly7_y_t = [poly7_y[i](t_) for t_ in t]
plt.plot(poly7_x_t, poly7_y_t)
    





first_line = ['duration', 'x^0', 'x^1', 'x^2', 'x^3', 'x^4', 'x^5', 'x^6', 'x^7', 'y^0', 'y^1', 'y^2', 'y^3', 'y^4', 'y^5', 'y^6', 'y^7', 'z^0', 'z^1', 'z^2', 'z^3', 'z^4', 'z^5', 'z^6', 'z^7', 'yaw^0', 'yaw^1', 'yaw^2', 'yaw^3', 'yaw^4', 'yaw^5', 'yaw^6', 'yaw^7']
mode = 'w'
with open(cwd + '/vehicle0.csv', mode = mode) as csvfile:
    writer = csv.writer(csvfile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    writer.writerow(first_line)
    
    # i = 1
    # if i == 1:
    for i in range(len(T)):
        px = poly7_x[i].coeffs.tolist()
        px.reverse()
        py = poly7_y[i].coeffs.tolist()
        py.reverse()
        pz = [0] * len(px)
        pj = [0] * len(px)
        writer.writerow([T[i]] + px + py + pz + pj)