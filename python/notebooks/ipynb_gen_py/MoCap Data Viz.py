
# coding: utf-8

# # This notebook is an attempt to get position from the sensor recordings taken during our motion capture tests

# In[1]:

import numpy as np
np.set_printoptions(precision=4, suppress=True)
import matplotlib.pyplot as plt
import csv
import json

# helper function
from IPython.display import Markdown, display
def print_h1(value):
    display(Markdown("# " + str(value)))
    
def print_h2(value):
    display(Markdown("## " + str(value)))
        
def print_h3(value):
    display(Markdown("### " + str(value)))

def print_md(value):
    display(Markdown(value))


# ## Read MoCap data and RoboRIO data form files

# In[2]:

data_dir = "../../recorded_sensor_data/mocap_12_10-03-30-00/"
sensor_file = data_dir + "sensor_data.csv"
mocap_file = data_dir + "mocap_data.csv"
extra_points_file = data_dir + "extra_points.json"
sensor_reader = csv.reader(open(sensor_file, 'r'))
extra_points = json.load(open(extra_points_file, 'r'))
mocap_reader = csv.reader(open(mocap_file, 'r'))

# placeholder for missing pieces of data (tags were not captured very reliably)
MISSING_DATA = -123456789

# read offset to center of robot
template_centroid = np.array(extra_points['template_centroid'])
robot_center = np.array(extra_points['template_centroid'])
robot_center_offset = robot_center - template_centroid 

# skip headers
headers = next(sensor_reader)
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
# sensor data is a Nx15 array. N is the number of data points
sensor_data = np.array(sensor_data)
print("sensor_data shape:", sensor_data.shape)
print("idx header")
for i, h in enumerate(headers):
    print(i, h)

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

# In[3]:

# these should be pretty darn close
print("Seconds of IMU data recorded: ", (sensor_data[-1][-1] - sensor_data[0][-1])/1000.0)
print("Seconds of MoCap recorded:", len(mocap_data) / 100)


# ## Plot Mocap Data by Axis

# In[4]:

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


# The above graphs illustrate that there was some strange behavior of the rotation measurements. It looks like wrap around

# ## Plot X/Y position from MoCap
# 
# This is what we will call our "ground truth" for the position of the robot over time.

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


# Here's what the data looks like in that weird "jump". Turns out it's not actually a "jump" but smooth transition

# In[7]:

print(mocap_states[2010:2030,2])


# ## Regard the first mocap pose as the "origin" for the dead reckoning methods
# 
# It's important to note that there are actually a bunch of points at the origin, then the origin jumps a few centimeters to another spot. I can't explain this but we'll just be aware of that as we compare against mocap.

# In[8]:

global_origin_x = mocap_states[0,0]
global_origin_y = mocap_states[0,1]
global_origin_yaw = mocap_states[0,2]
global_origin_xy = np.array([[global_origin_x], [global_origin_y]])
print("Global Origin", [global_origin_x, global_origin_y, global_origin_yaw])


# In[9]:

print_h2("Uncalibrated data:")
plt.plot(sensor_data[:,0], label="acc x")
plt.plot(sensor_data[:,1], label="acc y")
plt.plot(sensor_data[:,2], label="acc z")
plt.title("acc Data")
plt.legend(bbox_to_anchor=(1,1))
plt.ylabel("g")
plt.show()

plt.plot(sensor_data[:,3], label="Gyro x")
plt.plot(sensor_data[:,4], label="Gyro y")
plt.plot(sensor_data[:,5], label="Gyro z")
plt.title("Gyro Data")
plt.legend()
plt.ylabel("degrees/second")
plt.show()


# # Calibration
# 
# We first apply the results of our IMU calibration experiment. Truthfully, they have little effect other than accounting for the magical $0.4$ constant we've been using. It's also worth noting that these numbers come from the the _other_ NavX so it's not really fair to use them but I want to illustrate the whole procedure.

# In[10]:

def calibrate(input_data):
    # Ideal accelerometer calibration parameters
    acc_calibration_params = np.array([2.29299485e-03, 9.73357832e-04, 2.18891523e-03,
                                       9.97372417e-01, 9.98078141e-01, 9.95206414e-01,
                                       -8.12765191e-03, -1.24052008e-02, -1.41327621e-02])
    # Ideal gyro calibration parameters
    gyro_calibration_params = np.array([0.00844165, 0.00196508, -0.00410652,
                                       -0.01548789, -0.00191488, -0.00770839,
                                       0.40658698, 0.40275294, 0.40165824])
    Ta = np.array([[1, -acc_calibration_params[0], acc_calibration_params[1]],
                   [0, 1, -acc_calibration_params[2]],
                   [0, 0, 1]])
    Tg = np.array([[1, -gyro_calibration_params[0], gyro_calibration_params[1]],
                   [gyro_calibration_params[2], 1, -gyro_calibration_params[3]],
                   [-gyro_calibration_params[5], gyro_calibration_params[5], 1]])
    
    Ka = np.array([[acc_calibration_params[3], 0, 0],
                   [0, acc_calibration_params[4], 0],
                   [0, 0, acc_calibration_params[5]]])
    
    Kg = np.array([[gyro_calibration_params[6], 0, 0],
                   [0, gyro_calibration_params[7], 0],
                   [0, 0, gyro_calibration_params[8]]])

    ba = acc_calibration_params[6:9]
    bg = np.array([0.12964524, 0.15079131, 0.08211978])
    
    output_data = np.empty((input_data.shape[0], 6))
    output_data[:,0:3] = (Ta@Ka@(input_data[:,0:3] + ba).T).T
    output_data[:,3:6] = (Tg@Kg@(input_data[:,3:6] + bg).T).T
    
    return output_data
    
calibrated_imu_data = calibrate(sensor_data)


# In[11]:

print_h2("Calibrated data vs Raw Data")
plt.figure(figsize=(10,10))
plt.plot(sensor_data[:,0], label="acc x")
plt.plot(sensor_data[:,1], label="acc y")
plt.plot(sensor_data[:,2], label="acc z")
plt.plot(calibrated_imu_data[:,0], label="Calibrated acc x")
plt.plot(calibrated_imu_data[:,1], label="Calibrated acc y")
plt.plot(calibrated_imu_data[:,2], label="Calibrated acc z")
plt.title("acc Data")
plt.legend(bbox_to_anchor=(1,1))
plt.ylabel("g")
plt.show()

plt.figure(figsize=(10,10))
plt.plot(sensor_data[:,3], label="Gyro x")
plt.plot(sensor_data[:,4], label="Gyro y")
plt.plot(sensor_data[:,5], label="Gyro z")
plt.plot(calibrated_imu_data[:,3], label="Calibrated Gyro x")
plt.plot(calibrated_imu_data[:,4], label="Calibrated Gyro y")
plt.plot(calibrated_imu_data[:,5], label="Calibrated Gyro z")
plt.title("Gyro Data")
plt.legend()
plt.ylabel("degrees/second")
plt.show()


# # Base frame calibration

# In[80]:

def base_rotation(mean_acc_while_stationary):
    """ https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d """
    a = mean_acc_while_stationary
    b = np.array([0, 0, 1])
    v = np.cross(a, b)
    c = np.dot(a, b)
    v_x = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    R = np.eye(3) + v_x + (v_x@v_x)*(1/(1+c))
    return R
   
t_init = 100 # samples of the robot while stationary
stationary_accelerometer_data = calibrated_imu_data[:t_init,0:3]
stationary_mean_acc_raw = np.mean(stationary_accelerometer_data, axis=0)
stationary_mean_acc = stationary_mean_acc_raw / np.linalg.norm(stationary_mean_acc_raw)

# account for the yaw angle of the robot versus IMU (after it's rotated flat)
robot_to_imu_yaw = -np.pi/2
base_yaw_rot = np.array([[np.cos(robot_to_imu_yaw), -np.sin(robot_to_imu_yaw), 0],
                        [np.sin(robot_to_imu_yaw), np.cos(robot_to_imu_yaw), 0],
                        [0, 0, 1]])

R = base_yaw_rot @ base_rotation(stationary_mean_acc)

final_imu_data = np.empty(calibrated_imu_data.shape)
final_imu_data[:,0:3] = (R @ calibrated_imu_data[:,0:3].T).T
final_imu_data[:,3:6] = (R @ calibrated_imu_data[:,3:6].T).T

print_h2("Calibrated and Rotated versus Original Data")
print("some differences are visible, especially in the gyroscope")

plt.figure(figsize=(10,10))
plt.plot(sensor_data[:,0], label="acc x")
plt.plot(sensor_data[:,1], label="acc y")
plt.plot(sensor_data[:,2], label="acc z")
plt.plot(final_imu_data[:,0], label="final acc x")
plt.plot(final_imu_data[:,1], label="final acc y")
plt.plot(final_imu_data[:,2], label="final acc z")
plt.title("acc Data")
plt.legend(bbox_to_anchor=(1,1))
plt.ylabel("g")
plt.show()

plt.figure(figsize=(10,10))
plt.plot(sensor_data[:,3], label="Gyro x")
plt.plot(sensor_data[:,4], label="Gyro y")
plt.plot(sensor_data[:,5], label="Gyro z")
plt.plot(final_imu_data[:,3], label="final Gyro x")
plt.plot(final_imu_data[:,4], label="final Gyro y")
plt.plot(final_imu_data[:,5], label="final Gyro z")
plt.title("Gyro Data")
plt.legend()
plt.ylabel("degrees/second")
plt.show()


# # Integrating Navx Gyro to get Angle

# In[81]:

yaws = []
yaw = np.rad2deg(global_origin_yaw)
for data in final_imu_data:
    gyro_z = -data[5]  # the negative sign is to tf into mocap frame
    yaw += 0.02 * gyro_z
    yaws.append(yaw)
    
naive_yaws = []
og_yaw = np.rad2deg(global_origin_yaw)
for data in sensor_data:
    gyro_z = -data[5] * 0.4
    og_yaw += 0.02 * gyro_z
    naive_yaws.append(og_yaw)


# In[82]:

plt.plot(yaws, label="final integrated gyro")
plt.plot(naive_yaws, label="0.4 * raw integrated gyro")
# sample every other point (50hz versus 100hz collection) and convert to degrees
plt.plot(np.rad2deg(mocap_states[0:-1:2,2]), label='mocap')
plt.ylabel("degrees")
plt.title("Yaw of robot")
plt.legend()
plt.show()


# This suggests that the IMU calibration is not correct.
# A naive 0.4 and just as good as the calibrated and rotated data.
# It seems to make the intgrated yaw **worse**

# # Analysis of accurate of Integrating Yaw from NavX Gyro

# In[83]:

error = (np.rad2deg(mocap_states[0:-1:2,2])[:len(yaws)] - yaws[:len(yaws)])

# ignore outlies
error = error[:900]

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

print("from this it is not clear whether gyro drift plays a role in the error. However, error is within 5 degrees for the first 20 seconds.")


# # X/Y from NavX

# In[84]:

# adjust the NavX data to start with the correct global X, Y, and Yaw
navx_x = sensor_data[:,6] - sensor_data[0,6]
navx_y = sensor_data[:,7] - sensor_data[0,7]
navx_xy = np.vstack((navx_x, navx_y))

# subtract initial X/Y (idk why this is not just 0???)
global_yaw_rot = np.array([[np.cos(global_origin_yaw), -np.sin(global_origin_yaw)], [np.sin(global_origin_yaw), np.cos(global_origin_yaw)]])
navx_xy = (global_yaw_rot@navx_xy) + global_origin_xy


# # Encoder

# In[108]:

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


# In[109]:

plt.figure(figsize=(20,10))
plt.plot(sensor_data[:,9]*distance_per_pulse,label='left encoder rate (m/s)')
plt.plot(sensor_data[:,10]*distance_per_pulse, label='right encoder rate (m/s)')
plt.plot(-sensor_data[:,11],label='left joystick (-1 to 1)')
plt.plot(sensor_data[:,12], label='right joystick (-1 to 1)')
plt.title("encoder on MoCap robot")
plt.legend()
plt.show()


# ## Getting position from IMU
# 
# Consists of simply
#  - double integrate accelerometer
#  - integrate gyro Z axis

# In[110]:

def PositionFromIMU(imu_data, dt_s, x0, y0, yaw0):
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
    yaw_rad = yaw0
    for d in imu_data:
        a = d[:3]
        gyro_z = d[5]
        yaw_rad += dt_s * -np.deg2rad(gyro_z)
        yaw_rot = np.array([[np.cos(yaw_rad), -np.sin(yaw_rad), 0], [np.sin(yaw_rad), np.cos(yaw_rad), 0], [0, 0, 1]])
        a = yaw_rot @ a
        ax = a[0]
        ay = a[1]
        
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

# In[111]:

pva_from_imu = PositionFromIMU(final_imu_data, 0.02, global_origin_x, global_origin_y, global_origin_yaw)

plt.plot(pva_from_imu2[2], label="vxs")
plt.plot(pva_from_imu2[3], label="vys")
plt.title("velocities from IMU")
plt.legend()
plt.show()


# In[112]:

plt.figure(figsize=(15,15))
plt.title("Sensor Data versus MoCap")
TT = 400

# :2 accounts for the fact the the mocap samples twice as fast as our encoder/IMU data
plt.scatter(mocap_states[:TT*2:2,0], mocap_states[:TT*2:2,1], marker='.', s=2, color='r', label='MoCap')

plt.scatter(encoder_xs[:TT], encoder_ys[:TT], marker='.', s=2, color='k', label='encoders')
plt.scatter(navx_xy[0,:TT], navx_xy[1,:TT], marker='.', s=2, color='y', label="navx API")
plt.scatter(pva_from_imu[0][:TT], pva_from_imu[1][:TT], marker='.', s=1, color='b', label='IMU')
plt.axis("square")

plt.legend()
plt.show()


# # Testing on (original) Turtlebot accelerometer data
# 
# This data was collected carelessly, but over MXP so it should be usable. The navx may have shifted slightly while driving.

# In[90]:

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


# In[ ]:



