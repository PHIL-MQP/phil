
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
# Key prediction & update equations
# 
# $$ \hat{x}_{t} = Ax_t + Bu_t $$
# $$ P_{t+1} = AP_tA^T + Q $$
# $$ K_t = P_tC^T(CP_tC^T + R)^{-1} $$
# $$ x = \hat{x} + K(z_t - C\hat{x}_t) $$
# $$ P_t = (I - K_tC)P_t $$
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
# \Delta x_{t+1} &= \cos(\theta)\frac{1}{4}\Delta t^2*\alpha_l + \cos(\theta)\frac{1}{4}\Delta t^2*\alpha_r \\
# \Delta y_{t+1} &= \sin(\theta)\frac{1}{4}\Delta t^2*\alpha_l + \sin(\theta)\frac{1}{4}\Delta t^2*\alpha_r \\
# \Delta\theta_{t+1} &= \frac{R\Delta t^2}{W2}\alpha_l + \frac{-R\Delta t^2}{W2}\alpha_r\\
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
# \cos(\theta_t)\frac{1}{4}\Delta t^2 & \cos(\theta_t)\frac{1}{4}\Delta t^2 \\
# \sin(\theta_t)\frac{1}{4}\Delta t^2 & \sin(\theta_t)\frac{1}{4}\Delta t^2 \\
# \frac{R\Delta t^2}{2W} & \frac{-R\Delta t^2}{2W} \\
# \cos(\theta_t)\frac{1}{2}\Delta t & \cos(\theta_t)\frac{1}{2}\Delta t \\
# \sin(\theta_t)\frac{1}{2}\Delta t & \sin(\theta_t)\frac{1}{2}\Delta t \\
# \frac{R\Delta t}{W} & \frac{-R\Delta t}{W}\\
# 0 & 0 \\
# 0 & 0 \\
# 0 & 0 \\
# \end{bmatrix}
# \begin{bmatrix}
# \alpha_l \\
# \alpha_r \\
# \end{bmatrix}
# $$

# ## General matrix dimensions for multi-sensor multi-state stuffs
# 
# N = number of state variables
# M = number of control variables
# L = number of measurements from all sensors
# 
# |matrix|variable name|size|
# |------|-----------|----|
# | A | A | NxN |
# | B | B | NxM |
# | C | C | LxN |
# | K | K | NxL |
# | z | measurement | Lx1 |
# | x | priori/posterior_estimate | Nx1 |
# | P | estimate_covariance | NxN |
# | Q | process_covariance  | NxN |
# | R | measurement_covariance | LxL |

# ### The measurement Vector + Update bit
# 
# $$
# \begin{bmatrix}
# \text{Accelerometer }\ddot{x} \\
# \text{Accelerometer }\ddot{y} \\
# \text{Gyro }\dot{\theta} \\
# \text{Camera }x \\
# \text{Camera }y \\
# \text{Camera }\theta \\
# \text{Encoders }x \\
# \text{Encoders }y \\
# \text{Encoders }\theta \\
# \end{bmatrix} -
# \begin{bmatrix}
# 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 \\
# 0 & 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 \\
# 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 \\
# 1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
# 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
# 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 \\
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

# ## Questions
# 
#  - Does the $Bu$ part mean a change in state or full actual state? If it's full actual state then this makes no sense
#  - Should $u$ be $\alpha$ (wheel accelerations, which is assumed proportional to voltage) or $\omega$ (wheel speeds)?
#  - On the slides example, why is P [[0,0],[0,q_x]]? What's with the process noise model?

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
