
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
    
data = np.array(data)
accelerometer_data = data[:,4:7]
gyro_data = data[:,7:10] * np.pi / 180  # convert degrees to radians


# In[3]:

print(data[-1][-1] - data[0][-1])
print(data.shape)


# In[4]:

plt.figure(figsize=(10,10))
plt.title("Measured Acceleration")
plt.plot(accelerometer_data[:,0], label='acc x')
plt.plot(accelerometer_data[:,1], label='acc y')
plt.plot(accelerometer_data[:,2], label='acc z')
plt.legend()

plt.figure(figsize=(10,10))
plt.title("Measured Angular Rate")
plt.plot(gyro_data[:,0], label='gyro x/roll')
plt.plot(gyro_data[:,1], label='gyro y/pitch')
plt.plot(gyro_data[:,2], label='gyro z/yaw')
plt.legend()

plt.show()


# ## Generic Calibration
# 
# compensate for bias, scale, and misalignment

# In[5]:

calib_T = np.array([[1, 2.71075764e-03, 4.55981725e-03],[0, 1, 7.38354478e-04],[0,0,1]])
calib_K = np.array([[9.97279234e-01, 0, 0],[0, 9.96661774e-01, 0],[0, 0, 9.89959950e-01]])
calib_b = np.array([[-6.37606144e-03, -8.99928659e-03, -1.99175409e-02]])

calibrated_accelerometer_data = calib_T@calib_K@(accelerometer_data+calib_b).T

plt.figure(figsize=(10,10))
plt.plot(accelerometer_data[:,0], label='x')
plt.plot(calibrated_accelerometer_data[0,:], label='calibrated x')
plt.plot(accelerometer_data[:,1], label='y')
plt.plot(calibrated_accelerometer_data[1,:], label='calibrated y')
plt.plot(accelerometer_data[:,2], label='z')
plt.plot(calibrated_accelerometer_data[2,:], label='calibrated z')
plt.legend(bbox_to_anchor=(1,1))
plt.title("calibrate for bias, scale, and internal misalignment")
plt.show()

calibrated_accelerometer_data = calibrated_accelerometer_data.T


# ## Body to Accelerometer Frame Calibration

# The offset between the coordinate frame of the robot and the accelerometer can be described as a rotation in roll, pitch, and yaw. This rotation can be represented as a matrix multiplication, and we can use the measured acceleration in $x$, $y$, and $z$ to compute these angles. To do this, we assume that when the robot is stationary that the acceleration should be $0$ in $x$ and $y$, and $1$ in $z$. The observed accelerations will differ from these values if there is any misalignment between the IMU and the base frame.
# 
# https://math.stackexchange.com/a/476311

# In[6]:

def base_rotation(mean_acc_while_stationary):
    """ https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d """
    a = mean_acc_while_stationary
    b = np.array([0, 0, 1])
    v = np.cross(a, b)
    c = np.dot(a, b)
    v_x = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    R = np.eye(3) + v_x + (v_x@v_x)*(1/(1+c))
    return R


# In[7]:

t_init = 25 # ~25 is static
stationary_accelerometer_data = calibrated_accelerometer_data[:t_init]
stationary_mean_acc_raw = np.mean(stationary_accelerometer_data, axis=0)
stationary_mean_acc = stationary_mean_acc_raw / np.linalg.norm(stationary_mean_acc_raw)
print(np.linalg.norm(stationary_mean_acc_raw))


R = base_rotation(stationary_mean_acc)

print("raw", stationary_mean_acc_raw)
print("normalized", stationary_mean_acc)
print("adjusted", R@stationary_mean_acc)
print(R)


# In[8]:

rotated_acc_data = np.ndarray(calibrated_accelerometer_data.shape)
rotated_gyro_data = np.ndarray(gyro_data.shape)
for i, d in enumerate(calibrated_accelerometer_data):
    rotated_acc_data[i] = R@d
for i, d in enumerate(gyro_data):
    rotated_gyro_data[i] = R@d
    
plt.figure(figsize=(15,15))
plt.title("Rotated Acceleration")
plt.plot(calibrated_accelerometer_data[:,0], label='acc x')
plt.plot(calibrated_accelerometer_data[:,1], label='acc y')
plt.plot(calibrated_accelerometer_data[:,2], label='acc z')
plt.plot(rotated_acc_data[:,0], label='rot acc x')
plt.plot(rotated_acc_data[:,1], label='rot acc y')
plt.plot(rotated_acc_data[:,2], label='rot acc z')
plt.ylabel("m/s^2")
plt.legend()

plt.figure(figsize=(15,15))
plt.title("Rotated Gyro")
plt.plot(gyro_data[:,0], label='gyro x')
plt.plot(gyro_data[:,1], label='gyro y')
plt.plot(gyro_data[:,2], label='gyro z')
plt.plot(rotated_gyro_data[:,0], label='rot gyro x')
plt.plot(rotated_gyro_data[:,1], label='rot gyro y')
plt.plot(rotated_gyro_data[:,2], label='rot gyro z')
plt.ylabel("radians per second")
plt.legend()

plt.show()


# In[13]:

plt.figure(figsize=(5,5))
_yaw = []
yy = 0
for g in rotated_gyro_data:
    yy += g[2] * 0.01 * 8
    _yaw.append(yy)
    
plt.plot(_yaw)
plt.ylabel("radians")
plt.show()


# In[14]:

def DoubleIntegrateAccelerometer(acc_data, gyro_data, T, K, b, drift=np.zeros((3,1))):
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
    yaw = 0
    for idx, (a_s, g_s) in enumerate(zip(acc_data, gyro_data)):
        g_s_rot = R @ (g_s * 8)
        yaw += g_s_rot[2] * dt_s
        YawR = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        a_s_rot = YawR @ a_s
        a_o = T@K@((a_s_rot + b).T + idx * drift)
#         a_o = T@K@((a_s + b).T + idx * drift)
        ax = a_o[0][0]
        ay = a_o[1][0]
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


# In[15]:

raw = DoubleIntegrateAccelerometer(calibrated_accelerometer_data, gyro_data, np.eye(3), np.eye(3), np.zeros((1,3)))
means = R@(np.mean(calibrated_accelerometer_data[:t_init], axis=0))
bias = np.array([[means[0], means[1], 0]])
# drift = np.array([[.0005], [-0.0007], [0]])
drift = np.array([[0.], [0], [0]])
print("bias from t_init: ", bias)
print(R@(np.mean(calibrated_accelerometer_data[-t_init:], axis=0)))
manual_calib = DoubleIntegrateAccelerometer(calibrated_accelerometer_data, gyro_data, R, np.eye(3), np.array([[-0.0025, -0.02339312, 0]]), drift)
calib = DoubleIntegrateAccelerometer(calibrated_accelerometer_data, gyro_data, R, np.eye(3), bias, drift)

plt.figure(figsize=(15,5))
plt.plot(raw[4], label='raw ax')
plt.plot(raw[5], label='raw ay')
plt.plot(calib[4], label='calib ax')
plt.plot(calib[5], label='calib ay')
# plt.plot(manual_calib[4], label='manual_calib ax')
# plt.plot(manual_calib[5], label='manual_calib ay')
plt.legend()

plt.figure(figsize=(5,5))
plt.plot(raw[2], label='raw vx')
plt.plot(raw[3], label='raw vy')
plt.plot(calib[2], label='calib vx')
plt.plot(calib[3], label='calib vy')
# plt.plot(manual_calib[2], label='manual_calib vx')
# plt.plot(manual_calib[3], label='manual_calib vy')
plt.legend()

plt.figure(figsize=(15,15))
plt.scatter(raw[0], raw[1], marker='.', s=10, color='m', label='raw data')
plt.scatter(calib[0], calib[1], marker='.', s=10, color='k', label='calib')
plt.scatter(manual_calib[0], manual_calib[1], marker='.', s=10, color='g', label='rotated acc')
plt.legend()
plt.axis('square')
plt.show()

