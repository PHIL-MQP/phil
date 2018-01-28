
# coding: utf-8

# # Testing the USB C++ Linux Driver versus the MXP (i2c) C++ RoboRIO port

# In[3]:

import numpy as np
import matplotlib.pyplot as plt
import csv


# In[27]:

usb_reader = csv.reader(open("./recorded_sensor_data/imu_calibration/stationary_logs/bad_data_test_usb.csv"))
mxp_reader = csv.reader(open("./recorded_sensor_data/imu_calibration/stationary_logs/bad_data_test_mxp.csv"))

# skip headers
next(usb_reader)
next(usb_reader)
next(mxp_reader)
next(mxp_reader)

usb_data = []
mxp_data = []
for usb_line, mxp_line in zip(usb_reader, mxp_reader):
    usb_float = [float(u) for u in usb_line]
    mxp_float = [float(m) for m in mxp_line]
    usb_data.append(usb_float)
    mxp_data.append(mxp_float)
usb_data = np.array(usb_data)
mxp_data = np.array(mxp_data)
print(usb_data.shape)


# In[28]:

plt.figure(figsize=(10,10))
plt.plot(usb_data[:,0], label='usb x')
plt.plot(mxp_data[:,0], label='mxp x')
plt.plot(usb_data[:,1], label='usb y')
plt.plot(mxp_data[:,1], label='mxp y')
plt.plot(usb_data[:,2], label='usb z')
plt.plot(mxp_data[:,2], label='mxp z')
plt.legend(bbox_to_anchor=(1,1))

plt.figure(figsize=(10,10))
plt.plot(usb_data[:,3], label='usb roll')
plt.plot(mxp_data[:,3], label='mxp roll')
plt.plot(usb_data[:,4], label='usb pitch')
plt.plot(mxp_data[:,4], label='mxp pitch')
plt.plot(usb_data[:,5], label='usb yaw')
plt.plot(mxp_data[:,5], label='mxp yaw')
plt.legend(bbox_to_anchor=(1,1))
plt.show()


# In[ ]:



