
# coding: utf-8

# In[1]:

import numpy as np
import csv
import matplotlib.pyplot as plt
from scipy.optimize import root
from math import cos, sin


# In[2]:

reader = csv.reader(open("recorded_sensor_data/turtlebot_12_07_18-44-00/turtlebot_data.csv", 'r'))

data = []
next(reader)  # skip header
for idx, row in enumerate(reader):
    row = [float(d) for d in row]
    data.append(row)
#     if idx > 0 and data[idx-1][0] > 65536 and row[0] < 65536
data = np.array(data)
accelerometer_data = data[:,4:7]


# In[3]:

# compute bias (from components of gravity) from the initial static interval
t_init = 20
init_data = accelerometer_data[:20]
acc = np.mean(init_data, axis=0)
print(acc)


# In[4]:

# plt.figure(figsize=(15,15))
# plt.plot(data[:,0], label='left encoder ticks')
# plt.plot(data[:,1], label='right encoder ticks')
# plt.legend()
theta_acc = [-2.71075764e-03, 4.55981725e-03, -7.38354478e-04, 9.97279234e-01, 9.96661774e-01, 9.89959950e-01, -6.37606144e-03, -8.99928659e-03, -1.99175409e-02]  # from the latest data
# theta_acc = [-0.05175933, 0.00348791, 0.00268508, 0.99769397, 0.99724823, 0.99706769, -0.00595171, -0.01183456, -0.01726781] # from nicolette 2

T = np.array([[1, -theta_acc[0], theta_acc[1]], [0, 1, -theta_acc[2]], [0, 0, 1]])
K = np.array([[theta_acc[3], 0, 0], [0, theta_acc[4], 0], [0, 0, theta_acc[5]]])
b = np.array([[theta_acc[6], theta_acc[7], theta_acc[8]]])
fixed_accelerometer_data = np.ndarray(accelerometer_data.shape)

for i, a_s in enumerate(accelerometer_data):
    a_o = T@K@(a_s+b).T
    fixed_accelerometer_data[i] = a_o[0]


plt.plot(fixed_accelerometer_data[:,0], label='fixed acc x')
plt.plot(fixed_accelerometer_data[:,1], label='fixed acc y')
plt.plot(fixed_accelerometer_data[:,2], label='fixed acc z')
plt.plot(accelerometer_data[:,0], label='acc x')
plt.plot(accelerometer_data[:,1], label='acc y')
plt.plot(accelerometer_data[:,2], label='acc z')
plt.legend()

plt.show()


# In[5]:

def DoubleIntegrateAccelerometer(accelerometer_data, T, K, b, gravity_bias=np.array([ 0.08737485, -0.0150847])):
    x = 0
    y = 0
    vx = 0
    vy = 0
    dt_s = 0.01
    xs = []
    ys = []
    vxs = []
    vys = []
    axs = []
    ays = []
    for a_s in accelerometer_data:
        a_o = (T@K@(a_s + b).T)
        ax = a_o[0][0] - gravity_bias[0]
        ay = a_o[1][0] - gravity_bias[0]
        az = a_o[2][0]
        
        vx += ax * dt_s
        vy += ay * dt_s
        x += vx * dt_s + 0.5 * ax * dt_s ** 2
        y += vy * dt_s + 0.5 * ay * dt_s ** 2
        axs.append(ax)
        ays.append(ay)
        vxs.append(vx)
        vys.append(vy)
        xs.append(x)
        ys.append(y)
    
    return xs, ys, vxs, vys, axs, ays


# In[6]:

nothing = DoubleIntegrateAccelerometer(accelerometer_data, np.eye(3), np.eye(3), np.zeros((1,3)), np.zeros(3))
no_calib = DoubleIntegrateAccelerometer(accelerometer_data, np.eye(3), np.eye(3), np.zeros((1,3)))
calib = DoubleIntegrateAccelerometer(accelerometer_data, T, K, b)

plt.figure(figsize=(15,15))
plt.plot(nothing[2], label='raw vx')
plt.plot(nothing[3], label='raw vy')
plt.plot(no_calib[2], label='calib vx')
plt.plot(no_calib[3], label='calib vy')
plt.legend()

plt.figure(figsize=(15,15))
plt.scatter(nothing[0], nothing[1], marker='.', s=10, color='m', label='raw data')
plt.scatter(no_calib[0], no_calib[1], marker='.', s=10, color='g', label='corrected acc, no bias')
plt.scatter(calib[0], calib[1], marker='.', s=10, color='k', label='corrected acc, with bias')
plt.legend()
plt.show()


# In[13]:

initial_guess = np.array([0, 0, 0])  # alpha, beta, gamma
acc = 

def body_to_imu_tf(angles):
    c1 = cos(angle[0])
    c2 = cos(angle[1])
    c3 = cos(angle[2])
    s1 = sin(angle[0])
    s2 = sin(angle[1])
    s3 = sin(angle[2])
    
    x = acc[0]
    y = acc[1]
    z = acc[2]
    
    R = np.array([[c2*c3, -c2*s3, s2],[c1*s3+c3*s1*s2, c1*c3-s1*s2*s3, -c2*s1],[s1*s3-c1*c3*s2, c3*s1+c1*s2*s3, c1*c2]])
        
    df = np.array([[0, -s2*c3*x+s2*s3*y+c2*z, -c2*s3*x+-c2*c3*y],[],[]])
    
    return np.array([0,0,1]) - R@acc, df

root(body_to_imu_tf, initial_guess, jac=True, method='lm')


# In[ ]:



