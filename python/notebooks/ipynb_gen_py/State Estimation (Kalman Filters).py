
# coding: utf-8

# In[3]:

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
# $$ y_{navx} = \begin{bmatrix} \theta \\ \text{Encoder}_r \\ \text{Encoder}_l \\ \end{bmatrix} $$
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
# $$ x = \hat{x} + K(z_t - C\hat{x}_t) $$
# $$ P_t = (I - K_tC)P_t $$
# $$ y_t = Hx_t $$
# 
# We must describe our dynamics. How do we compute our next state given our current state and the control inputs?
# 
# \begin{align}
# x_{t+1} &= x_t + \dot{x}_t\Delta t + \tfrac{1}{2}\ddot{x}_t\Delta t^2 \\
# y_{t+1} &= y_t + \dot{y}_t\Delta t + \tfrac{1}{2}\ddot{y}_t\Delta t^2 \\
# \theta_{t+1} &= \theta_t + \dot{\theta}_t\Delta t + \tfrac{1}{2}\ddot{\theta}_t\Delta t^2\\
# \dot{x}_{t+1} &= \dot{x}_t + \ddot{x}_t\Delta t \\
# \dot{y}_{t+1} &= \dot{y}_t + \ddot{y}_t\Delta t \\
# \dot{\theta}_{t+1} &= \dot{\theta}_t+\ddot{\theta}_t\Delta t \\
# \ddot{x}_{t+1} &= a_x \\
# \ddot{y}_{t+1} &= a_y \\
# \ddot{\theta}_{t+1 &= \tan^{-1}(a_y, a_x) \\
# \end{align}
# 
# In order to use en EKF, we need all the partial derivatives of these equations with respect to each of the state variables. We represent these partials as the Jacobian, so it's in a matrix. Each row contains all the partials for each state variable equation (in the same order as above).
# 
# $$\begin{bmatrix}
# 1 & 0 & 0 & \Delta t & 0 & 0 & 0.5\Delta t^2 & 0 & 0 \\
# 0 & 1 & 0 & 0 & \Delta t & 0 & 0 & 0.5\Delta t^2 & 0 \\
# 0 & 0 & 1 & 0 & 0 & \Delta t & 0 & 0 & 0.5\Delta t^2 \\
# 0 & 0 & 0 & 1 & 0 & 0 & \Delta t & 0 & 0 \\
# 0 & 0 & 0 & 0 & 1 & 0 & 0 & \Delta t & 0 \\
# 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & \Delta t \\
# 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
# 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
# 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
# \end{bmatrix}$$

# To add encoders to our state space equations we use our simple model
# 
# \begin{align*}
# r &= \text{wheel radius} \\
# \begin{bmatrix} \dot{y} \\ \dot{\theta} \\ \end{bmatrix}
# &=
# \frac{r}{\alpha W} \begin{bmatrix} \frac{\alpha W}{2} & \frac{\alpha W}{2} \\ -1 & 1 \\ \end{bmatrix}
# \begin{bmatrix}\text{Encoder}_l \\ \text{Encoder}_r \\ \end{bmatrix}
# \begin{bmatrix} \dot{y} \\ \dot{\theta} \\ \end{bmatrix} \\
# \begin{bmatrix} \dot{y} \\ \dot{\theta} \\ \end{bmatrix}
# &=
# \begin{bmatrix} \frac{r}{2} & \frac{r}{2} \\ \frac{-r}{\alpha W} & \frac{r}{\alpha W} \\ \end{bmatrix}
# \begin{bmatrix}\text{Encoder}_l \\ \text{Encoder}_r \\ \end{bmatrix} \\
# \begin{bmatrix}\text{Encoder}_l \\ \text{Encoder}_r \\ \end{bmatrix}
# &=
# \begin{bmatrix} \frac{r}{2} & \frac{r}{2} \\ \frac{-r}{\alpha W} & \frac{r}{\alpha W} \\ \end{bmatrix}^{-1}
# \begin{bmatrix} \dot{y} \\ \dot{\theta} \\ \end{bmatrix} \\
# \text{det} &= \frac{r^2}{2\alpha W} - \frac{-r^2}{2\alpha W} \\
#            &= \frac{r^2}{\alpha W} \\
# \begin{bmatrix}\text{Encoder}_l \\ \text{Encoder}_r \\ \end{bmatrix}
# &=
# \frac{\alpha W}{r^2}\begin{bmatrix} \frac{r}{\alpha W} & \frac{r}{\alpha W} \\ \frac{-r}{2} & \frac{r}{2} \\ \end{bmatrix}
# \begin{bmatrix} \dot{y} \\ \dot{\theta} \\ \end{bmatrix} \\
# \begin{bmatrix}\text{Encoder}_l \\ \text{Encoder}_r \\ \end{bmatrix}
# &=
# \begin{bmatrix} \frac{1}{r} & \frac{1}{r} \\ \frac{-\alpha W}{2r} & \frac{\alpha W}{2r} \\ \end{bmatrix}
# \begin{bmatrix} \dot{y} \\ \dot{\theta} \\ \end{bmatrix} \\
# \end{align*}
# 

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

# ### The measurement Vector + Update bit
# 
# The KF update step for the sensor data coming from the RoboRIO
# 
# $$
# \begin{bmatrix}
# \theta \\
# \text{Encoder}_l \\
# \text{Encoder}_r \\
# \end{bmatrix} -
# \begin{bmatrix}
# 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 \\
# 0 & 0 & 0 & 0 & \frac{1}{r} & \frac{1}{r} & 0 & 0 & 0 \\
# 0 & 0 & 0 & 0 & \frac{-\alpha W}{2r} & \frac{\alpha W}{2r} & 0 & 0 & 0\\
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

# ### The process covariance matrix
# 
# $$
# \begin{bmatrix}
# W_1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
# 0 & W_2 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
# 0 & 0 & W_3 & 0 & 0 & 0 & 0 & 0 & 0 \\
# 0 & 0 & 0 & W_4 & 0 & 0 & 0 & 0 & 0 \\
# 0 & 0 & 0 & 0 & W_5 & 0 & 0 & 0 & 0 \\
# 0 & 0 & 0 & 0 & 0 & W_6 & 0 & 0 & 0 \\
# 0 & 0 & 0 & 0 & 0 & 0 & W_7 & 0 & 0 \\
# 0 & 0 & 0 & 0 & 0 & 0 & 0 & W_8 & 0 \\
# 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & W_9 \\
# \end{bmatrix}
# $$

# Resources:
# 
# https://www.researchgate.net/post/Extended_Kalman_Filter_with_asynchronous_measurements
