
# coding: utf-8

# # Kalman Filter
# A linear kalman filter. First we define our state variables x
# 
# $$ x = \begin{bmatrix}
# x & y & \theta &
# \dot{x} & \dot{y} & \dot{\theta} \end{bmatrix}^T $$
# 
# The control inputs are the wheel velocities.
# 
# $$ u = \begin{bmatrix}
# w_l \\
# w_r \\
# \alpha_l \\
# \alpha_r \\
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
# \end{align}
# 
# The other values in x are not a function of x, but some are a function of u.
# 
# \begin{align}
# \Delta x_{t+1} &= \cos(\theta)\frac{1}{2}\Delta t*w_l + \cos(\theta)\frac{1}{2}\Delta t*w_r + \cos(\theta)\frac{1}{4}\Delta t^2*\alpha_l + \cos(\theta)\frac{1}{4}\Delta t^2*\alpha_r \\
# \Delta y_{t+1} &= \sin(\theta)\frac{1}{2}\Delta t*w_l + \sin(\theta)\frac{1}{2}\Delta t*w_r + \sin(\theta)\frac{1}{4}\Delta t^2*\alpha_l + \sin(\theta)\frac{1}{4}\Delta t^2*\alpha_r \\
# \Delta\theta_{t+1} &= \frac{R\Delta t}{W}w_l+\frac{-R\Delta t}{W}w_r + \frac{R\Delta t^2}{W2}\alpha_l + \frac{-R\Delta t^2}{W2}\alpha_r\\
# \Delta\dot{x}_{t+1} &= \cos(\theta_t)\frac{\alpha_l+\alpha_r}{2}\Delta t \\
# \Delta\dot{y}_{t+1} &= \sin(\theta_t)\frac{\alpha_l+\alpha_r}{2}\Delta t \\
# \Delta\dot{\theta}_{t+1} &= \frac{R\Delta t}{W}\alpha_l + \frac{-R\Delta t}{W}\alpha_r \\
# \end{align}
# 
# The equations derived above come from these simpler facts:
# 
# \begin{align}
# \ddot{x}_{t+1} &= \cos(\theta_t)\frac{\alpha_l+\alpha_r}{2} \\
# \ddot{y}_{t+1} &= \sin(\theta_t)\frac{\alpha_l+\alpha_r}{2} \\
# \ddot{\theta}_{t+1} &= \frac{R}{W} * \alpha_l + \frac{-R}{W} * \alpha_r \\
# \end{align}

# ## If state is just position and velocity and acceleration
# 
# $$
# \begin{bmatrix}
# x_{t+1} \\
# y_{t+1} \\
# \theta_{t+1} \\
# \dot{x}_{t+1} \\
# \dot{y}_{t+1} \\
# \dot{\theta}_{t+1} \\
# \ddot{x}_{t+1} \\
# \ddot{y}_{t+1} \\
# \ddot{\theta}_{t+1} \\
# \end{bmatrix} =
# \begin{bmatrix}
# 1 & 0 & 0 & \Delta t & 0 & 0 & \tfrac{1}{2}\Delta t& 0 & 0 \\
# 0 & 1 & 0 & 0 & \Delta t & 0 & 0 & \tfrac{1}{2}\Delta t& 0 \\
# 0 & 0 & 1 & 0 & 0 & \Delta t & 0 & 0 & \tfrac{1}{2}\Delta t \\
# 0 & 0 & 0 & 1 & 0 & 0 & \Delta t & 0 & 0 \\
# 0 & 0 & 0 & 0 & 1 & 0 & 0 & \Delta t & 0 \\
# 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & \Delta t \\
# 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 \\
# 0 & 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 \\
# 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 1 \\
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
# \end{bmatrix} +
# \begin{bmatrix}
# \cos(\theta)\frac{1}{2}\Delta t & \cos(\theta)\frac{1}{2}\Delta t & \cos(\theta)\frac{1}{4}\Delta t^2 & \cos(\theta)\frac{1}{4}\Delta t^2 \\
# \sin(\theta)\frac{1}{2}\Delta t & \sin(\theta)\frac{1}{2}\Delta t & \sin(\theta)\frac{1}{4}\Delta t^2 & \sin(\theta)\frac{1}{4}\Delta t^2 \\
# \frac{R\Delta t}{W} & \frac{-R\Delta t}{W} & \frac{R\Delta t^2}{2W} & \frac{-R\Delta t^2}{2W} \\
# 0 & 0 & \cos(\theta)\frac{1}{2}\Delta t & \cos(\theta)\frac{1}{2}\Delta t \\
# 0 & 0 & \sin(\theta)\frac{1}{2}\Delta t & \sin(\theta)\frac{1}{2}\Delta t \\
# 0 & 0 & \frac{R\Delta t}{W} & \frac{-R\Delta t}{W}\\
# 0 & 0 & 0 & 0 \\
# 0 & 0 & 0 & 0 \\
# 0 & 0 & 0 & 0 \\
# \end{bmatrix}
# \begin{bmatrix}
# w_l \\
# w_r \\
# \alpha_l \\
# \alpha_r \\
# \end{bmatrix}
# $$

# ## Questions for Michalson
# 
#  - Does the $Bu$ part mean a change in state or full actual state? If it's full actual state then this makes no sense
#  - Should $u$ be $\alpha$ (wheel accelerations, which is assumed proportional to voltage) or $\omega$ (wheel speeds)?

# If we include acceleration and bias/drift in our state vector we get a bunch of useless 1's
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
# 
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

# In[ ]:



