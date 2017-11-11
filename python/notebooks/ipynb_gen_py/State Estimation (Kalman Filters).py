
# coding: utf-8

# # Kalman Filter
# A linear kalman filter. First we define our state variables x
# 
# $$ x = \begin{bmatrix}
# x & y & \theta &
# \dot{x} & \dot{y} & \dot{\theta} &
# \ddot{x} & \ddot{y} & \ddot{\theta} &
# a_x^d & a_y^d & a_z^d &
# a_x^b & a_y^b & a_z^b & \end{bmatrix}^T $$
# 
# The control inputs are the wheel velocities.
# 
# $$ u = \begin{bmatrix}
# w_l \\
# w_r \\
# \end{bmatrix} $$
# 
# We must describe our dynamics. How do we compute our next state given our current state and the control inputs?
# 
# \begin{align}
# x_{t+1} &= x_t + \dot{x}_t\Delta t + \tfrac{1}{2}\ddot{x}_t\Delta t^2 \\
# y_{t+1} &= y_t + \dot{y}_t\Delta t + \tfrac{1}{2}\ddot{y}_t\Delta t^2 \\
# \theta_{t+1} &= \theta_t + \dot{\theta}_t\Delta t + \tfrac{1}{2}\ddot{\theta}_t\Delta t^2 \\
# \dot{x}_{t+1} &= \dot{x}_t + \ddot{x}_t\Delta t \\
# \dot{y}_{t+1} &= \dot{y}_t + \ddot{y}_t\Delta t \\
# \dot{\theta}_{t+1} &= \dot{\theta}_t + \ddot{\theta}_t\Delta t \\
# \ddot{x}_{t+1} &= \ddot{x}_t \\ 
# \ddot{y}_{t+1} &= \ddot{y}_t \\
# \ddot{\theta}_{t+1} &= \ddot{\theta}_t \\
# \end{align}
# 
# The other values in x are not a function of x, but some are a function of u.
# 
# \begin{align}
# x_{t+1} &= R\cos\Big(\frac{(\bar{v}_r-\bar{v}_l)\Delta t}{W}-\bar{\theta}\Big)\frac{\Delta t}{W} * w_l +
#           -R\cos\Big(\frac{(\bar{v}_r-\bar{v}_l)\Delta t}{W}-\bar{\theta}\Big)\frac{\Delta t}{W} * w_r \\
# y_{t+1} &= -R\sin\Big(\frac{(\bar{v}_r-\bar{v}_l)\Delta t}{W}-\bar{\theta}\Big)\frac{\Delta t}{W} * w_l +
#            R\sin\Big(\frac{(\bar{v}_r-\bar{v}_l)\Delta t}{W}-\bar{\theta}\Big)\frac{\Delta t}{W} * w_r \\
# \theta_{t+1} &= \frac{\Delta t}{R-\frac{W}{2}} * w_l \\
# \dot{x}_{t+1} &= \frac{1}{2} \cos(\theta) * w_l + \frac{1}{2} \cos(\theta) * w_r \\
# \dot{y}_{t+1} &= \frac{1}{2} \sin(\theta) * w_l + \frac{1}{2} \sin(\theta) * w_r \\
# \dot{\theta}_{t+1} &= \frac{1}{W} * w_l + \frac{-1}{W} * w_r \\
# \ddot{x}_{t+1} &= \frac{\dot{x}_{t+1} - \dot{x}_t}{\Delta t} \\
# \ddot{y}_{t+1} &= \frac{\ddot{y}_{t+1} - \ddot{y}_t}{\Delta t} \\
# \ddot{\theta}_{t+1} &= \frac{\ddot{\theta}_{t+1} - \ddot{\theta}_t}{\Delta t}\\
# \end{align}
# 
# We write this in the matrix form of $x=Ax+Bu$
# 
# $$
# \begin{bmatrix}
# x_{t+1} \\ y_{t+1} \\ \theta_{t+1} \\
# \dot{x}_{t+1} \\ \dot{y}_{t+1} \\ \dot{\theta}_{t+1} \\
# \ddot{x}_{t+1} \\ \ddot{y}_{t+1} \\ \ddot{\theta}_{t+1} \\
# d^x_{t+1} \\ d^y_{t+1} \\ d^z_{t+1} \\
# b^x_{t+1} \\ b^y_{t+1} \\ b^z_{t+1} \end{bmatrix} =
# \begin{bmatrix}
# 1 & 0 & 0 & \Delta t & 0 & 0 & \tfrac{1}{2}\Delta t& 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
# 0 & 1 & 0 & 0 & \Delta t & 0 & 0 & \tfrac{1}{2}\Delta t& 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
# 0 & 0 & 1 & 0 & 0 & \Delta t & 0 & 0 & \tfrac{1}{2}\Delta t& 0 & 0 & 0 & 0 & 0 & 0 \\
# 0 & 0 & 0 & 1 & 0 & 0 & \Delta t & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
# 0 & 0 & 0 & 0 & 1 & 0 & 0 & \Delta t & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
# 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & \Delta t & 0 & 0 & 0 & 0 & 0 & 0 \\
# 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
# 0 & 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
# 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 \\
# 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 \\
# 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 \\
# 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 \\
# 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 \\
# 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 \\
# 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 1 \\
# \end{bmatrix}
# \begin{bmatrix}
# x \\ y \\ \theta \\ \dot{x} \\ \dot{y} \\ \dot{\theta} \\ \ddot{x} \\ \ddot{y} \\ \ddot{\theta}
# \\ a_x^d \\ a_y^d \\ a_z^d \\ a_x^b \\ a_y^b \\ a_z^b \\ \end{bmatrix} +
# \begin{bmatrix}
# R\cos\Big(\frac{(\bar{v}_r-\bar{v}_l)\Delta t}{W}-\bar{\theta}\Big)\frac{\Delta t}{W} &
# -R\cos\Big(\frac{(\bar{v}_r-\bar{v}_l)\Delta t}{W}-\bar{\theta}\Big)\frac{\Delta t}{W} \\
# -R\sin\Big(\frac{(\bar{v}_r-\bar{v}_l)\Delta t}{W}-\bar{\theta}\Big)\frac{\Delta t}{W} &
# R\sin\Big(\frac{(\bar{v}_r-\bar{v}_l)\Delta t}{W}-\bar{\theta}\Big)\frac{\Delta t}{W} \\
# \frac{\Delta t}{R-\frac{W}{2}} & 0 \\
# \frac{1}{2} \cos(\theta) & \frac{1}{2} \cos(\theta) \\
# \frac{1}{2} \sin(\theta) & \frac{1}{2} \sin(\theta) \\
# \frac{1}{W} & \frac{-1}{W} \\
# ? & ? \\
# ? & ? \\
# ? & ? \\
# ? & ? \\
# ? & ? \\
# ? & ? \\
# ? & ? \\
# ? & ? \\
# ? & ? \\
# \end{bmatrix}
# \begin{bmatrix}
# w_l \\
# w_r \\
# \end{bmatrix}
# $$

# What the heck should C be? C transforms the measurements into state.
# 
# Equations for acceleration. First remove the components of x/y due to misaligned axis, then apply scaling correction, then bias
# $$x_{t+1} = x_t + \frac{1}{2}\Delta t^2(\alpha_x*s_x+b_x)$$
# $$y_{t+1} = y_t + \frac{1}{2}\Delta t^2(\alpha_y*s_y+b_y)$$
# 
# $$
# x = 
# \begin{bmatrix}
# \frac{1}{2}\Delta t^2 & \frac{1}{2}\Delta t^2 & 0 \\
# \end{bmatrix}
# \begin{bmatrix}
# acc_x \\
# acc_y \\
# acc_z \\
# \end{bmatrix}
# $$

# ## Simpler version with simpler state
# 
# $$
# \begin{bmatrix}
# x_{t+1} \\ y_{t+1} \\ \theta_{t+1} \\
# \dot{x}_{t+1} \\ \dot{y}_{t+1} \\ \dot{\theta}_{t+1} \end{bmatrix} =
# \begin{bmatrix}
# 1 & 0 & 0 & \Delta t & 0 & 0 & \tfrac{1}{2}\Delta t& 0 & 0 \\
# 0 & 1 & 0 & 0 & \Delta t & 0 & 0 & \tfrac{1}{2}\Delta t& 0 \\
# 0 & 0 & 1 & 0 & 0 & \Delta t & 0 & 0 & \tfrac{1}{2}\Delta t \\
# 0 & 0 & 0 & 1 & 0 & 0 & \Delta t & 0 & 0 \\
# 0 & 0 & 0 & 0 & 1 & 0 & 0 & \Delta t & 0 \\
# 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & \Delta t \\
# \end{bmatrix}
# \begin{bmatrix}
# x \\ y \\ \theta \\ \dot{x} \\ \dot{y} \\ \dot{\theta} \\ \end{bmatrix} +
# \begin{bmatrix}
# R\cos\Big(\frac{(\bar{v}_r-\bar{v}_l)\Delta t}{W}-\bar{\theta}\Big)\frac{\Delta t}{W} &
# -R\cos\Big(\frac{(\bar{v}_r-\bar{v}_l)\Delta t}{W}-\bar{\theta}\Big)\frac{\Delta t}{W} \\
# -R\sin\Big(\frac{(\bar{v}_r-\bar{v}_l)\Delta t}{W}-\bar{\theta}\Big)\frac{\Delta t}{W} &
# R\sin\Big(\frac{(\bar{v}_r-\bar{v}_l)\Delta t}{W}-\bar{\theta}\Big)\frac{\Delta t}{W} \\
# \frac{\Delta t}{R-\frac{W}{2}} & 0 \\
# \frac{1}{2} \cos(\theta) & \frac{1}{2} \cos(\theta) \\
# \frac{1}{2} \sin(\theta) & \frac{1}{2} \sin(\theta) \\
# \frac{1}{W} & \frac{-1}{W} \\
# \end{bmatrix}
# \begin{bmatrix}
# w_l \\
# w_r \\
# \end{bmatrix}
# $$

# # Particle Filter
# 
#  - Take the current pose estimate of the robot
#  - Sample points $X=[x^0_t, \dots, x^m_t]$ around it (guess positions near your current estimate)
#  - Take measurement of various sensors $z_t = [z^0_t, \dots, z^n_t]$
#  - For each sample:
#    - Compute the likelihood of your sensor readings given that guess of position  $w^i_t = p(z_t \mid x^i_t)$
#    - This is done using your sensor models
#  - Keep each sample with a probability equal to w
#  - To get one number, take a weighted average of all particles (weighted by their probabilities)
