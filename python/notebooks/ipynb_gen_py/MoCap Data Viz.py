
# coding: utf-8

# In[4]:

import numpy as np
import matplotlib.pyplot as plt
import csv


# In[75]:

data_dir = "../../recorded_sensor_data/mocap_11_17_18-00-00/"
imu_file = data_dir + "mocap_imu_encoder_data.csv"
mocap_file = data_dir + "MQP_FRC_Trial 4.csv"
imu_reader = csv.reader(open(imu_file, 'r'))
mocap_reader = csv.reader(open(mocap_file, 'r'))

# skip headers
next(imu_reader)
next(mocap_reader)
next(mocap_reader)
next(mocap_reader)
next(mocap_reader)
next(mocap_reader)

imu_data = []
for imu_row in imu_reader:
    data = [float(d) for d in imu_row]
    if len(data) > 0 and data[-1] > 1800:
        imu_data.append(data)
imu_data = np.array(imu_data)

mocap_data = []
for mocap_row in mocap_reader:
    data = [float(d) for d in mocap_row]
    if len(data) > 0:
        mocap_data.append(data)
mocap_data = np.array(mocap_data)


# In[76]:

def yawdiff(y1, y2):
    diff = y2 - y1
    if diff > np.pi:
        return diff - np.pi * 2;
    if diff < -np.pi:
        return diff + np.pi * 2;
    return diff;


# In[77]:

print("Seconds of IMU data recorded: ", imu_data[-1][-1] - imu_data[0][-1])
print("Seconds of MoCap recorded:", len(mocap_data) / 100)


# In[78]:

plt.plot(mocap_data[:,2], label="rx")
plt.plot(mocap_data[:,3], label="ry")
plt.plot(mocap_data[:,4], label="rz")
plt.legend()
plt.show()

plt.plot(mocap_data[:,5], label="tx")
plt.plot(mocap_data[:,6], label="ty")
plt.plot(mocap_data[:,7], label="tz")
plt.legend()
plt.show()


# In[103]:

states = np.ndarray((mocap_data.shape[0], 9))
state = np.zeros(9)
for mocap_idx in range(1, len(mocap_data)):
    data = mocap_data[mocap_idx]
    last_data = mocap_data[mocap_idx - 1]
    rx = data[2]
    ry = data[3]
    rz = data[4]
    tx = data[5]
    ty = data[6]
    tz = data[7]
    state[0] = tx
    state[1] = ty
    state[2] += yawdiff(rz, last_data[4]) # handles wrap-around
    states[mocap_idx] = state


# In[116]:

plt.plot(states[:,2], label='theta')
plt.legend()
plt.title("Unwrapped Angle")

plt.figure()
plt.scatter(states[:,0], states[:,1], marker='.', s=1, color='r')
plt.show()


# In[ ]:



