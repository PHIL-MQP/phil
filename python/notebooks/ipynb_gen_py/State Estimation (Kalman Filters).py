
# coding: utf-8

# In[2]:


import numpy as np
import matplotlib.pyplot as plt


# # Extended Kalman Filter
# A non-linear extended kalman filter. First we define our state variables x
# 
# $$ x = \begin{bmatrix}
# x & y & \theta &
# \dot{x} & \dot{y} & \dot{\theta} &
# \ddot{x} & \ddot{y} & \ddot{\theta} \end{bmatrix}^T$$
# 
# On possible choice is to use the measured accelerations as the control inputs. We get these accelerations from the NavX by calling `GetWorldLinearX/Y()`
# 
# $$ u = \begin{bmatrix}
# a_x \\
# a_y \\
# \end{bmatrix} $$
# 
# Our measurement vectors
# 
# We assume m/s^2 for acceleration, radians for theta, and m/s (linear speed) for encoder speeds
# $$ y_{navx} = \begin{bmatrix} \theta \\ v_r \\ v_l \\ \end{bmatrix} $$
# 
# $$ y_{beacon} = \begin{bmatrix} \text{Beacon}_x \\ \text{Beacon}_y \\ \end{bmatrix} $$
# 
# $$ y_{camera} = \begin{bmatrix} \text{Camera}_x \\ \text{Camera}_y \\ \text{Camera}_\theta \\ \end{bmatrix} $$
# 
# Key prediction & update equations
# 
# $$ \hat{x}_{t} = f(x_t , u_t) $$
# $$ P_{t+1} = AP_tA^T + Q $$
# $$ K_t = P_tC^T(CP_tC^T + R)^{-1} $$
# $$ y_t = g(x_t) $$
# $$ x = \hat{x} + K_t(z_t - y_t) $$
# $$ P_t = (I - y_t)P_t  = P_t - y_tP_t $$
# 
# We must describe our dynamics. How do we compute our next state given our current state and the control inputs? Here we have $W$ which is the track with and $\alpha$ which is a slip factor. This comes from Yu et. al 2011 (Dynamic modeling and power modeling of robotic skid-steered wheeled vehicles). $\alpha\in[1,\infty]$. where $1$ means no slip, and greater than one means more slip.
# 
# Overall were are trying to come up with the equations that make up this:
# $$ \hat{x}_{t+1} = f(x_t, u_t) $$
# 
# \begin{align}
# v &= \frac{v_l + v_r}{2} \\
# x_{t+1} &= x_t + \dot{x}_t\Delta t + \tfrac{1}{2}\ddot{x}_t\Delta t^2 \\
# y_{t+1} &= y_t + \dot{y}_t\Delta t + \tfrac{1}{2}\ddot{y}_t\Delta t^2 \\
# \theta_{t+1} &= \theta_t + \dot{\theta}_t\Delta t \\
# \dot{x}_{t+1} &= v\cos(\theta_t) \\
# \dot{y}_{t+1} &= v\sin(\theta_t) \\
# \dot{\theta}_{t+1} &= \frac{v_r - v_l}{\alpha W} \\
# \ddot{x}_{t+1} &=  \ddot{x}_{t} \\
# \ddot{y}_{t+1} &=  \ddot{y}_{t} \\
# \ddot{\theta}_{t+1} &= 0 \\
# \end{align}
# 
# I'm not sure about the last three equations...
# 
# In order to use en EKF, we need all the partial derivatives of these equations with respect to each of the state variables. We represent these partials as the Jacobian, so it's in a matrix. Each row contains all the partials for each state variable equation (in the same order as above).
# 
# $$\begin{bmatrix}
# 1 & 0 & 0 & \Delta t & 0 & 0 & 0.5\Delta t^2 & 0 & 0 \\
# 0 & 1 & 0 & 0 & \Delta t & 0 & 0 & 0.5\Delta t^2 & 0 \\
# 0 & 0 & 1 & 0 & 0 & \Delta t & 0 & 0 & 0 \\
# 0 & 0 & -v\sin(\theta_t) & 0 & 0 & 0 & 0 & 0 & 0 \\
# 0 & 0 & v\cos(\theta_t) & 0 & 0 & 0 & 0 & 0 & 0 \\
# 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
# 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 \\
# 0 & 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 \\
# 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
# \end{bmatrix}$$

# ### The measurement Vector + Update bit
# 
# In these cases, we can write have linear measurment equations because our measurements are all values we have in our state space.
# 
# The KF update step for the sensor data coming from the RoboRIO.
# 
# **note**: In order for the subtraction between $\theta$ and predicted $\theta$ to work, we must use _unwrapped_ angles (still radians)!
# 
# $$
# \begin{bmatrix}
# a_x \\
# a_y \\
# \theta \\
# \end{bmatrix} -
# \begin{bmatrix}
# 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 \\
# 0 & 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 \\
# 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 \\
# \end{bmatrix}
# \begin{bmatrix}
# x \\
# y \\
# \theta \\
# \dot{x} \\
# \dot{y} \\
# \dot{\theta} \\
# \ddot{x} \\
# \ddot{y} \\
# \ddot{\theta} \\
# \end{bmatrix}
# $$
# 
# update step for the beacons
# 
# $$
# \begin{bmatrix}
# \text{Beacons}_x \\
# \text{Beacons}_y \\
# \end{bmatrix} -
# \begin{bmatrix}
# 1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
# 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
# \end{bmatrix}
# \begin{bmatrix}
# x \\
# y \\
# \theta \\
# \dot{x} \\
# \dot{y} \\
# \dot{\theta} \\
# \ddot{x} \\
# \ddot{y} \\
# \ddot{\theta} \\
# \end{bmatrix}
# $$
# 
# update step for the camera
# 
# $$
# \begin{bmatrix}
# \text{Camera}_x \\
# \text{Camera}_y \\
# \text{Camera}_\theta \\
# \end{bmatrix} -
# \begin{bmatrix}
# 1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
# 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
# 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 \\
# \end{bmatrix}
# \begin{bmatrix}
# x \\
# y \\
# \theta \\
# \dot{x} \\
# \dot{y} \\
# \dot{\theta} \\
# \ddot{x} \\
# \ddot{y} \\
# \ddot{\theta} \\
# \end{bmatrix}
# $$
# 
# Because we are receiving our sensor updates asynchronously, we will just run these updates steps as soon as the data from the three sources is available. Basically this means we will have a seperate thread for collecting camera images and calling the camera update step, and another for talking to the PSoC over serial.

# ## General matrix dimensions for multi-sensor multi-state stuffs
# 
# N = number of state variables
# 
# M = number of control variables
# 
# L = number of measurements from all sensors
# 
# 
# |matrix|variable name|size|
# |------|-----------|----|
# | A | A | NxN |
# | B | B | NxM |
# | C | C | LxN |
# | K | K | NxL |
# | z | measurement | Lx1 |
# | x | priori/posterior_estimate | Nx1 |
# | u | control input | Mx1 |
# | P | estimate_covariance | NxN |
# | Q | process_covariance  | NxN |
# | R | measurement_covariance | LxL |

# # Alternative EKF Formulation
# 
# The above formulation has the wheel speeds as measured by encoders as the control input $u$ and the accelerometer readings as a measurement. We could also reverse these roles, which results in the equations desrcibed below.

# \begin{align}
# x_{t+1} &= x_t + \dot{x}_t\Delta t + \tfrac{1}{2}\ddot{x}_t\Delta t^2 \\
# y_{t+1} &= y_t + \dot{y}_t\Delta t + \tfrac{1}{2}\ddot{y}_t\Delta t^2 \\
# \theta_{t+1} &= \text{yaw} \\
# \dot{x}_{t+1} &= \dot{x}_t + \ddot{x}_t\Delta t \\
# \dot{y}_{t+1} &= \dot{y}_t + \ddot{y}_t\Delta t \\
# \dot{\theta}_{t+1} &= 0 \\
# \ddot{x}_{t+1} &=  a_x \\
# \ddot{y}_{t+1} &= a_y \\
# \ddot{\theta}_{t+1} &= 0 \\
# \end{align}
# 
# Here we need some source of $\theta$, so we use the yaw measurement from the NavX? Not sure if this is a good idea?
# 
# Notice this is a linear in the state, so we can rewrite it in matrix form.
# 
# $$
# \begin{bmatrix}
# x_{t+1} \\
# y_{t+1} \\ 
# \dot{x}_{t+1} \\
# \dot{y}_{t+1} \\
# \dot{\theta}_{t+1} \\
# \ddot{x}_{t+1} \\
# \ddot{y}_{t+1} \\
# \ddot{\theta}_{t+1} \\
# \end{bmatrix}
# =\begin{bmatrix}
# x_{t+1} \\
# y_{t+1} \\ 
# \dot{x} \\
# \dot{y} \\
# \dot{\theta} \\
# \ddot{x} \\
# \ddot{y} \\
# \ddot{\theta} \\
# \end{bmatrix}
# $$
# 
# The KF update step for the sensor data coming from the RoboRIO is now different, since now our wheels speeds $v_l$ and $v_r$ are measurements. We start with two of our kinematics equations and re-arrange to solve for $v_l$ and $v_r$ as functions of our state variables $\dot{x}$, $\dot{\theta}$,  and $\theta$.
# 
# \begin{align}
# \dot{x} &= \frac{v_l+v_r}{2}\cos(\theta) \\
# \frac{2\dot{x}}{\cos(\theta)} &= v_l+v_r \\
# \frac{2\dot{x}}{\cos(\theta)} - v_r &= v_l \\
# \dot{\theta} &= \frac{v_l-v_r}{\alpha W} \\
# \alpha W\dot{\theta} &= v_l-v_r \\
# \alpha W\dot{\theta} + v_r &= v_l \\
# \alpha W\dot{\theta} + v_r &= \frac{2\dot{x}}{\cos(\theta)} - v_r \\
# 2v_r &= \frac{2\dot{x}}{\cos(\theta)} - \alpha W\dot{\theta} \\
# v_r &= \frac{\dot{x}}{\cos(\theta)} - \frac{\alpha W\dot{\theta}}{2} \\
# v_l &= \alpha W\dot{\theta} + \frac{\dot{x}}{\cos(\theta)} - \frac{\alpha W\dot{\theta}}{2} = \frac{\dot{x}}{\cos(\theta)} + \frac{\alpha W\dot{\theta}}{2} \\
# \end{align}
# 
# We define the measurement vector in this formulation to be ordered:
# $$\begin{bmatrix} v_l \\ v_r \\ \theta \\ \end{bmatrix}$$

# It's important to note that now we have a non-linear measurement model. I think this is fine although I haven't found any examples of it in the literature.

# And like before we need our jacobian:
# 
# \begin{bmatrix}
# 0 & 0 & \frac{2\sin(\theta)}{\cos(\theta) + 1} & \frac{1}{\cos(\theta)} & \frac{-\alpha W}{2} & 0 & 0 & 0 & 0 \\
# 0 & 0 & \frac{2\sin(\theta)}{\cos(\theta) + 1} & \frac{1}{\cos(\theta)} & \frac{\alpha W}{2} & 0 & 0 & 0 & 0 \\
# 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 \\
# \end{bmatrix}

# ## Notes on BFL Implementation
# 
# the covariance update equation $ P_t = (I - y_t)P_t  = P_t - y_tP_t $ is implemented on lines 106-108 in `filter/kalman_filter.cpp`. `_Sigma_temp` is $P_t$, and `_Sigma_temp_par` is $y_t$.

# Resources:
# 
# https://www.researchgate.net/post/Extended_Kalman_Filter_with_asynchronous_measurements
