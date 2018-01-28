
# coding: utf-8

# In[1]:

import numpy as np
np.set_printoptions(precision=4, suppress=True)
import matplotlib.pyplot as plt
import csv
import json


# # Read MoCap data and RoboRIO data form files

# In[2]:

data_dir = "../../recorded_sensor_data/mocap_12_10-03-30-00/"
sensor_file = data_dir + "sensor_data.csv"
mocap_file = data_dir + "mocap_data.csv"
extra_points_file = data_dir + "extra_points.json"
sensor_reader = csv.reader(open(sensor_file, 'r'))
extra_points = json.load(open(extra_points_file, 'r'))
mocap_reader = csv.reader(open(mocap_file, 'r'))

MISSING_DATA = -123456789

# read offset to center of robot
template_centroid = np.array(extra_points['template_centroid'])
robot_center = np.array(extra_points['template_centroid'])
robot_center_offset = robot_center - template_centroid 

# skip headers
next(sensor_reader)
for i in range(5):
    next(mocap_reader)

sensor_data = []
for sensor_row in sensor_reader:
    data = [float(d) for d in sensor_row]
    # FYI: negate encoder rates
    mask = np.ones(len(data))
    mask[10] = -1
    data = data * mask
    sensor_data.append(data)
sensor_data = np.array(sensor_data)

mocap_data = []
for mocap_row in mocap_reader:
    data = []
    for d in mocap_row:
        try:
            data.append(float(d))
        except ValueError:
            data.append(MISSING_DATA)
    mocap_data.append(data)
mocap_data = np.array(mocap_data)


# ### Check the amount of data between the two matches?

# In[43]:

# these should be pretty darn close
print("Seconds of IMU data recorded: ", (sensor_data[-1][-1] - sensor_data[0][-1])/1000.0)
print("Seconds of MoCap recorded:", len(mocap_data) / 100)


# # Plot Mocap Data by Axis

# In[44]:

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


# # Plot X/Y position from MoCap

# In[5]:

mocap_states = np.ndarray((mocap_data.shape[0], 3))
mocap_states[0] = np.array([mocap_data[0,0], mocap_data[0,1], mocap_data[0,2]])
for mocap_idx in range(1, len(mocap_data)):
    data = mocap_data[mocap_idx]
    last_data = mocap_data[mocap_idx - 1]
    rx = data[2]
    ry = data[3]
    rz = data[4]
    tx = (data[5] + robot_center_offset[0]) / 1000
    ty = (data[6] + robot_center_offset[1]) / 1000
    tz = (data[7] + robot_center_offset[2]) / 1000
    mocap_states[mocap_idx-1][0] = tx
    mocap_states[mocap_idx-1][1] = ty
    mocap_states[mocap_idx-1][2] = rz
    


# In[6]:

plt.figure(figsize=(10,10))
plt.scatter(mocap_states[0,0], mocap_states[0,1], marker='.', s=100, color='b')
plt.scatter(mocap_states[:,0], mocap_states[:,1], marker='.', s=1, color='r')
plt.axis("square")
plt.show()

plt.figure(figsize=(10,10))
plt.plot(mocap_states[:,2], label='yaw')
plt.title("mocap yaw")
plt.show()


# ### Here's what the data looks like in that weird "jump". Turns out it's not actually a "jump" but smooth transition

# In[45]:

print(mocap_states[2010:2030,2])


# ## Regard the first mocap pose as the "origin" for the dead reckoning methods

# In[46]:

global_origin_x = mocap_states[0,0]
global_origin_y = mocap_states[0,1]
global_origin_yaw = mocap_states[0,2]
global_origin_xy = np.array([[global_origin_x], [global_origin_y]])
print("Global Origin", [global_origin_x, global_origin_y, global_origin_yaw])


# In[9]:

plt.plot(sensor_data[:,0], label="acc x")
plt.plot(sensor_data[:,1], label="acc y")
plt.plot(sensor_data[:,2], label="acc z")
plt.title("acc Data")
plt.legend(bbox_to_anchor=(1,1))
plt.ylabel("g")
plt.show()


# In[10]:

plt.plot(sensor_data[:,3], label="Gyro x")
plt.plot(sensor_data[:,4], label="Gyro y")
plt.plot(sensor_data[:,5], label="Gyro z")
plt.title("Gyro Data")
plt.legend()
plt.ylabel("degrees/second")
plt.show()


# # Base frame calibration

# In[78]:

def base_rotation(mean_acc_while_stationary):
    """ https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d """
    a = mean_acc_while_stationary
    b = np.array([0, 0, 1])
    v = np.cross(a, b)
    c = np.dot(a, b)
    v_x = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    R = np.eye(3) + v_x + (v_x@v_x)*(1/(1+c))
    return R
   
t_init = 100 # samples
stationary_accelerometer_data = sensor_data[:t_init,0:3]
stationary_mean_acc_raw = np.mean(stationary_accelerometer_data, axis=0)
stationary_mean_acc = stationary_mean_acc_raw / np.linalg.norm(stationary_mean_acc_raw)

R = base_rotation(stationary_mean_acc)

rotated_acc_data = np.ndarray((sensor_data.shape[0], 3))
rotated_gyro_data = np.ndarray((sensor_data.shape[0], 3))
corrected_acc = (R @ sensor_data[:,0:3].T).T
corrected_gyro = (R @ sensor_data[:,3:6].T).T
# the differences is too small to notice so we don't plot them


# # Integrating Navx Gyro to get Angle

# In[79]:

yaws = []
yaw = np.rad2deg(global_origin_yaw)
last_t = sensor_data[0][-1]
for data in sensor_data:
    gyro_z = data[5] * -0.4
    dt_s = (data[-1] - last_t)/1000
    yaw += dt_s * gyro_z
    yaws.append(yaw)
    last_t = data[-1]


# In[80]:

plt.plot(yaws, label="integrated gyro")
# sample every other point (50hz versus 100hz collection) and convert to degrees
plt.plot(np.rad2deg(mocap_states[0:-1:2,2]), label='mocap')
plt.ylabel("degrees")
plt.title("Yaw of robot")
plt.show()


# # Analysis of accurate of Integrating Yaw from NavX Gyro

# In[100]:

error = (np.rad2deg(mocap_states[0:-1:2,2])[:len(yaws)] - yaws[:len(yaws)])

# ignore outlies
for idx, e in enumerate(error):
    if abs(e) > 25:
        error[idx] = 0

average_error_by_second = []
for i in range(1, error.shape[0] // 20):
    average_error_by_second.append(np.mean(abs(error[(i-1)*20:i*20])))

plt.plot(error)
plt.xlabel("sample #")
plt.ylabel("degrees")
plt.title("Error between Gyro integration and Mocap data")
plt.show()

plt.figure()
plt.plot(average_error_by_second)
plt.plot([5]*len(average_error_by_second))
plt.title("averge error by second")
plt.xlabel("time (seconds)")
plt.ylabel("average error in degrees during that second")
plt.show()


# # X/Y from NavX

# In[13]:

# adjust the NavX data to start with the correct global X, Y, and Yaw
navx_x = sensor_data[:,6] - sensor_data[0,6]
navx_y = sensor_data[:,7] - sensor_data[0,7]
navx_xy = np.vstack((navx_x, navx_y))

# subtract initial X/Y (idk why this is not just 0???)
global_yaw_rot = np.array([[np.cos(global_origin_yaw), -np.sin(global_origin_yaw)], [np.sin(global_origin_yaw), np.cos(global_origin_yaw)]])
navx_xy = (global_yaw_rot@navx_xy) + global_origin_xy


# # Encoder

# In[14]:

# encoder kinematics
encoder_x = global_origin_x
encoder_y = global_origin_y
encoder_yaw = mocap_states[0][2]
encoder_xs = []
encoder_ys = []
alpha = 1.0
wheel_radius_m = 0.074
track_width_m = 0.9
dt_s = 0.05

distance_per_pulse = 0.000357 # measured on mocap robot

for data in sensor_data:
    wl = float(data[9]) * distance_per_pulse
    wr = float(data[10]) * distance_per_pulse
    
    B = alpha * track_width_m
    T = wheel_radius_m / B * np.array([[B / 2.0, B / 2.0], [-1, 1]])
    dydt, dpdt = T @ np.array([wl, wr])
    encoder_x = encoder_x + np.cos(encoder_yaw) * dydt * dt_s
    encoder_y = encoder_y + np.sin(encoder_yaw) * dydt * dt_s
    encoder_yaw += dpdt * dt_s
    
    encoder_xs.append(encoder_x)
    encoder_ys.append(encoder_y)


# In[15]:

plt.figure(figsize=(20,10))
plt.plot(sensor_data[:,9]*distance_per_pulse,label='left encoder rate (m/s)')
plt.plot(sensor_data[:,10]*distance_per_pulse, label='right encoder rate (m/s)')
plt.plot(-sensor_data[:,11],label='left joystick (-1 to 1)')
plt.plot(sensor_data[:,12], label='right joystick (-1 to 1)')
plt.title("encoder on MoCap robot")
plt.legend()
plt.show()


# ## Double Integrating Accelerometer

# In[16]:

def DoubleIntegrate(accelerometer_data, yaws, K, T, b, dt_s, x0, y0):
    x = x0
    y = y0
    vx = 0
    vy = 0
    xs = []
    ys = []
    vxs = []
    vys = []
    axs = []
    ays = []
    for a_s, yaw in zip(accelerometer_data, yaws):
        yaw_rad = yaw * np.pi / 180
        yaw_rot = np.array([[np.cos(yaw_rad), -np.sin(yaw_rad), 0], [np.sin(yaw_rad), np.cos(yaw_rad), 0], [0, 0, 1]])
        
        a_s_rot = yaw_rot @ a_s
        
        a_o = T@K@(a_s_rot+b).T
        ax = a_o[0][0]
        ay = a_o[1][0]
        
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


# ## Double Integrate Mocap Robot Sensor Data

# In[17]:

means = np.mean(sensor_data[:100],axis=0)
print("Average Accel X value:", means[0])
print("Average Accel Y value:", means[1])
print("Average Accel Z value:", means[2])

b = np.array([[-means[0], -means[1], 0]])
no_bias = DoubleIntegrate(sensor_data[:,3:6], 
                          yaws,
                          np.eye(3),
                          np.eye(3),
                          np.array([[0,0,0]]),
                          dt_s,
                          global_origin_x,
                          global_origin_y)
# calib = DoubleIntegrate(sensor_data[:,3:6],
#                         yaws,ca
#                         np.eye(3),
#                         np.eye(3),
#                         b,
#                         dt_s,
#                         global_origin_x,
#                         global_origin_y)

plt.plot(no_bias[2], label="no bias vxs")
plt.plot(no_bias[3], label="no bias vys")
# plt.plot(calib[2], label="calibrated vxs")
# plt.plot(calib[3], label="calibrated vys")
plt.title("velocities from integrating accelerometer")
plt.legend()
plt.show()


# In[18]:

print(mocap_states[0:7,0])
print(mocap_states[0:7:2,0])


# In[19]:

plt.figure(figsize=(15,15))
plt.title("Sensor Data versus MoCap")
TT = 400

# :2 accounts for the fact the the mocap samples twice as fast as our encoder/IMU data
plt.scatter(mocap_states[:TT:2,0], mocap_states[:TT:2,1], marker='.', s=2, color='r', label='MoCap')
# plt.scatter(encoder_xs[:TT], encoder_ys[:TT], marker='.', s=2, color='k', label='encoders')

plt.scatter(navx_xy[0,:TT], navx_xy[1,:TT], marker='.', s=2, color='y', label="navx API")
# plt.scatter(no_bias[0][:TT], no_bias[1][:TT], marker='.', s=1, color='b', label='Accelerometer, no bias')
# plt.scatter(calib[0][:TT], calib[1][:TT], marker='.', s=1, color='g', label='Accelerometer, with bias')
plt.axis("square")

plt.legend()
plt.show()


# ## Testing on Turtlebot accelerometer data

# In[20]:

turtlebot_dir = "../../recorded_sensor_data/turtlebot_original/"
data_file = turtlebot_dir + "interpolated_data.csv"
reader = csv.reader(open(data_file, 'r'))

tb_accelerometer_data = []
tb_encoder_x = 0
tb_encoder_y = 0
tb_encoder_yaw = 0.5
tb_encoder_xs = []
tb_encoder_ys = []
tb_alpha = 1.0
tb_wheel_radius_m = 0.038
tb_track_width_m = 0.23
tb_dt_s = 0.1
tb_yaws = []
tb_yaw = 0

next(reader)
for data in reader:
    wl = float(data[0])
    wr = float(data[1])
    ax = float(data[2])
    ay = float(data[3])
    t = float(data[-1])
    tb_gyro_z = float(data[7]) * 0.4
    
    tb_accelerometer_data.append([ax, ay, t])
    B = tb_alpha * tb_track_width_m
    T = tb_wheel_radius_m / B * np.array([[B / 2.0, B / 2.0], [-1, 1]])
    dydt, dpdt = T @ np.array([wl, wr])
    tb_encoder_x = tb_encoder_x + np.cos(tb_encoder_yaw) * dydt * tb_dt_s
    tb_encoder_y = tb_encoder_y + np.sin(tb_encoder_yaw) * dydt * tb_dt_s
    tb_encoder_yaw += dpdt * tb_dt_s
    
    tb_yaw += tb_dt_s * tb_gyro_z
    
    tb_encoder_xs.append(tb_encoder_x)
    tb_encoder_ys.append(tb_encoder_y)
    tb_yaws.append(tb_yaw)
    
init_means = np.mean(tb_accelerometer_data[:40], axis=0)
full_means = np.mean(tb_accelerometer_data, axis=0)
print("Average Init Accel X value:", init_means[0])
print("Average Init Accel Y value:", init_means[1])
print("Average Accel X value:", full_means[0])
print("Average Accel Y value:", full_means[1])

plt.plot(tb_yaws, label="integrated gyro")
plt.ylabel("degrees")
plt.title("Turtlebot Yaw")
plt.show()

no_bias = DoubleIntegrate(tb_accelerometer_data, tb_yaws, np.eye(3), np.eye(3), np.zeros((1,3)), dt_s=tb_dt_s)
K = np.eye(3)
T = np.eye(3)
b = np.array([[-init_means[0], -init_means[1], 0]])
init_calib = DoubleIntegrate(tb_accelerometer_data, tb_yaws, K, T, b, dt_s=tb_dt_s)
b = np.array([[-full_means[0], -full_means[1], 0]])
full_calib = DoubleIntegrate(tb_accelerometer_data, tb_yaws, K, T, b, dt_s=tb_dt_s)
b = np.array([[0.0096, -0.0052, 0]])
guess_calib = DoubleIntegrate(tb_accelerometer_data, tb_yaws, K, T, b, dt_s=tb_dt_s)

plt.figure(figsize=(10,10))
plt.scatter(tb_encoder_xs, tb_encoder_ys, marker='.', s=2, color='r', label='Encoder Data')
plt.scatter(no_bias[0], no_bias[1], marker='o', s=1, color='y', label='Accelerometer, no bias')
plt.scatter(init_calib[0], init_calib[1], marker='o', s=1, color='g', label='Accelerometer, init means')
plt.scatter(full_calib[0], full_calib[1], marker='o', s=1, color='k', label='Accelerometer, full means')
plt.scatter(guess_calib[0], guess_calib[1], marker='o', s=1, color='b', label='Accelerometer, guessed bias')
plt.title("Accelerometer versus Encoder (Turtlebot)")
plt.axis("square")
plt.legend(prop={'size': 10})
plt.show()

plt.figure(figsize=(10,10))
plt.plot(no_bias[4], label="no bias axs")
plt.plot(no_bias[5], label="no bias ays")
plt.plot(init_calib[4], label="calibrated axs")
plt.plot(init_calib[5], label="calibrated ays")
plt.title("Accelerations")
plt.legend()
plt.show()

plt.figure(figsize=(10,10))
plt.plot(no_bias[2], label="no bias vxs")
plt.plot(no_bias[3], label="no bias vys")
plt.plot(init_calib[2], label="calibrated vxs")
plt.plot(init_calib[3], label="calibrated vys")
plt.title("velocities from integrating accelerometer")
plt.legend()
plt.show()


# # Comparing Aruco localization to Mocap

# In[ ]:

import cv2


# In[ ]:

img_dir = "../../recorded_sensor_data/practice_image_processing/"
vid_file = img_dir + "out.avi"
vid = cv2.VideoCapture(vid_file)
img_timestamp_file = img_dir + "frame_time_stamps.csv"
img_timestamp_reader = csv.reader(open(img_timestamp_file))
camera_xs = []
camera_ys = []
for timestamp in img_timestamp_reader:
    t = float(timestamp[0])
    ok, frame = vid.read()
    
    # (t, frame)
    # detect markers in frame
    
    # compute position relative to markers
    camera_x = 0
    camera_y = 0
    
    camera_xs.append(camera_x)
    camera_ys.append(camera_y)
    
    if not ok:
        break


# In[ ]:

plt.figure(figsize=(10,10))
plt.scatter(camera_xs, camera_ys, marker='.', s=1, color='b', label='camera')
plt.scatter(mocap_states[:,0], mocap_states[:,1], marker='.', s=1, color='r', label='MoCap')
plt.title("Camera versus MoCap")
plt.legend()
plt.show()


# ## Comparing TimeStamp Accuracy between NavX and RoboRIO (FPGA)

# In[ ]:

dts_fpgas = []
dts_navxs = []
for i in range(1000):
    dt_s_fpga = (sensor_data[i+1][-1] - sensor_data[i][-1]) / 1000
    dt_s_navx = (sensor_data[i+1][-2] - sensor_data[i][-2])
    dts_fpgas.append(dt_s_fpga)
    dts_navxs.append(dt_s_navx)
    
plt.plot(dts_fpgas, label='fpga')
plt.plot(dts_navxs, label='navx')
plt.ylabel("delt t (seconds)")
plt.legend()
plt.title("Sensor reading timestamps")
plt.show()

