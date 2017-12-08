
# coding: utf-8

# In[9]:

import numpy as np
import csv
import matplotlib.pyplot as plt


# In[4]:

reader = csv.reader(open("recorded_sensor_data/turtlebot_12_07_18-44-00/turtlebot_data.csv", 'r'))

data = []
next(reader)  # skip header
for idx, row in enumerate(reader):
    row = [float(d) for d in row]
    data.append(row)
    if idx > 0 and data[idx-1][0] > 65536 and row[0] < 65536
data = np.array(data)


# In[8]:

plt.figure(figsize=(15,15))
plt.plot(data[:,0], label='left encoder ticks')
plt.plot(data[:,1], label='right encoder ticks')
plt.legend()

plt.figure(figsize=(15,15))
plt.plot(data[:,4], label='acc x')
plt.plot(data[:,5], label='acc y')
plt.plot(data[:,6], label='acc z')
plt.legend()

plt.show()


# In[ ]:



