
# coding: utf-8

# # Practice Field Data Analysis

# In[1]:

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from IPython.display import HTML
import csv


# ## Read in the data from 5x4 teleop
# 
# this was one of the trials that actually recorded properly

# In[2]:

reader = csv.reader(open("./recorded_sensor_data/field_data_1/5x4_teleop/mocap_data-839.451.csv", 'r'))  # not actually mocap data
headers = next(reader)
sensor_data = []
for row in reader:
    data = [float(d) for d in row]
    sensor_data.append(data)
    
# sensor data is a Nx15 array. N is the number of data points
sensor_data = np.array(sensor_data)
print("sensor_data shape:", sensor_data.shape)
print("idx header")
for i, h in enumerate(headers):
    print(i, h)


# ## Integrate gyro Z to get yaw

# In[3]:

yaws = []
yaw = 0
for data in sensor_data:
    gyro_z = -data[5]  # the negative sign is to tf into mocap frame
    yaw += 0.02 * gyro_z
    yaws.append(yaw)


# In[4]:

fig, ax = plt.subplots(figsize=(8,5))
ax.set_title("Yaw of robot")

cursor, = ax.plot([], [], lw=2)
def init():
    ax.plot(yaws, label='yaw')
    ax.set_xlabel('sample idx')
    ax.set_ylabel("degrees")
    cursor.set_data([0, 0], ax.get_ylim())
    return (cursor,)

scale = 10
def animate(i):
    x = [i*scale, i*scale]
    cursor.set_data(x, ax.get_ylim())
    return (cursor,)

anim = animation.FuncAnimation(fig, animate, frames=len(yaws)//scale, init_func=init, interval=scale*21)
html = anim.to_html5_video()
html += '<video width="320" height="240" controls><source src="./recorded_sensor_data/field_data_1/5x4_teleop/out_01_01_01-17-23.mp4" type="video/mp4"></video>'
HTML(html)


# If you play this along side the video of the robot you can see they seem quite reasonable. 

# # Sensor data from the second round of testing
# 
# The data recorded in these tests is different from the data recorded in the first test.

# In[5]:

reader = csv.reader(open("./recorded_sensor_data/field_data_2/drive_3/mocap_data-57.122.csv", 'r'))  # not actually mocap data
headers = next(reader)
sensor_data = []
for row in reader:
    data = [float(d) for d in row]
    sensor_data.append(data)
    
# sensor data is a Nx15 array. N is the number of data points
sensor_data = np.array(sensor_data)
print("sensor_data shape:", sensor_data.shape)
print("idx header")
for i, h in enumerate(headers):
    print(i, h)


# In[ ]:



