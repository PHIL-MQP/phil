
# coding: utf-8

# # Kalman Filter
# A linear kalman filter. First we define our state variables x
# 
# $$ x = \begin{bmatrix}
# x & y & \theta &
# \dot{x} & \dot{y} & \dot{\theta} &
# \ddot{x} & \ddot{y} & \ddot{\theta} \end{bmatrix}^T $$
# 
# The control inputs are the wheel accelerations (which we assume are a function of motor.set() values, such as those from a joystick
# 
# $$ u = \begin{bmatrix}
# \alpha_l \\
# \alpha_r \\
# \end{bmatrix} $$
# 
# Our measurement vectors
# 
# We assume m/s^2 for acceleration, radians for theta, and m/s (linear speed) for encoder speeds
# $$ y_{navx} = \begin{bmatrix} a_x \\ a_y \\ \theta \\ \text{Encoder}_r \\ \text{Encoder}_l \\ \end{bmatrix} $$
# 
# $$ y_{beacon} = \begin{bmatrix} \text{Beacon}_x \\ \text{Beacon}_y \\ \end{bmatrix} $$
# 
# $$ y_{camera} = \begin{bmatrix} \text{Camera}_x \\ \text{Camera}_y \\ \text{Camera}_\theta \\ \end{bmatrix} $$
# 
# Key prediction & update equations
# 
# $$ \hat{x}_{t} = Ax_t + Bu_t $$
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
# \theta_{t+1} &= \theta_t + \dot{\theta}_t\Delta t + \tfrac{1}{2}\ddot{\theta}_t\Delta t^2 \\
# \dot{x}_{t+1} &= \dot{x}_t + \ddot{x}_t\Delta t \\
# \dot{y}_{t+1} &= \dot{y}_t + \ddot{y}_t\Delta t \\
# \dot{\theta}_{t+1} &= \dot{\theta}_t + \ddot{\theta}_t\Delta t \\
# \end{align}
# 
# The other values in x are not a function of x, but some are a function of u.
# 
# \begin{align}
# \ddot{x}_{t+1} &= \cos(\theta_t)\frac{\alpha_l+\alpha_r}{2}\Delta t \\
# \ddot{y}_{t+1} &= \sin(\theta_t)\frac{\alpha_l+\alpha_r}{2}\Delta t \\
# \ddot{\theta}_{t+1} &= \frac{\alpha_r - \alpha_l}{W}\Delta t \\
#                    &= \frac{-\Delta t}{W}\alpha_l + \frac{\Delta t}{W}\alpha_l \\
# \end{align}

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

# ## The State-Space Equations
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
# 1 & 0 & 0 & \Delta t & 0 & 0 & \tfrac{1}{2}\Delta t^2& 0 & 0 \\
# 0 & 1 & 0 & 0 & \Delta t & 0 & 0 & \tfrac{1}{2}\Delta t^2& 0 \\
# 0 & 0 & 1 & 0 & 0 & \Delta t & 0 & 0 & \tfrac{1}{2}\Delta t^2 \\
# 0 & 0 & 0 & 1 & 0 & 0 & \Delta t & 0 & 0 \\
# 0 & 0 & 0 & 0 & 1 & 0 & 0 & \Delta t & 0 \\
# 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & \Delta t \\
# 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 \\
# 0 & 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 \\
# 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 1 \\
# \end{bmatrix}
# \begin{bmatrix}
# x_t \\
# y_t \\
# \theta_t \\
# \dot{x}_t \\
# \dot{y}_t \\
# \dot{\theta}_t \\
# \ddot{x}_t \\
# \ddot{y}_t \\
# \ddot{\theta}_t \\
# \end{bmatrix} +
# \begin{bmatrix}
# 0 & 0 \\
# 0 & 0 \\
# 0 & 0 \\
# \cos(\theta_t)\frac{1}{2}\Delta t & \cos(\theta_t)\frac{1}{2}\Delta t \\
# \sin(\theta_t)\frac{1}{2}\Delta t & \sin(\theta_t)\frac{1}{2}\Delta t \\
# \frac{-\Delta t}{W} & \frac{\Delta t}{W}\\
# 0 & 0 \\
# 0 & 0 \\
# 0 & 0 \\
# \end{bmatrix}
# \begin{bmatrix}
# \alpha_{l,t} \\
# \alpha_{r,t} \\
# \end{bmatrix}
# $$

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
# a_x \\
# a_y \\
# \theta \\
# \text{Encoder}_l \\
# \text{Encoder}_r \\
# \end{bmatrix} -
# \begin{bmatrix}
# 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 \\
# 0 & 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 \\
# 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 \\
# 0 & 0 & 0 & 0 & \frac{1}{r} & \frac{1}{r} & 0 & 0 & 0 \\
# 0 & 0 & 0 & 0 & \frac{-\alpha W}{2r} & \frac{\alpha W}{2r} & 0 & 0 & 0 \\
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

# $$ K_t = (P_tC^T)(CP_tC^T + R)^{-1} $$
# $$ K_t(CP_tC^T + R) = (P_tC^T)(CP_tC^T + R)^{-1}(CP_tC^T + R) $$
# $$ K_t(CP_tC^T + R) = (P_tC^T) $$
# $$ (CP_tC^T + R)^TK_t^T = (P_tC^T)^T $$
# 
# Now we have it in the form  $Ax=b$ where $A = (CP_tC^T + R)^T$, $x=k^T$ and $B=(P_tC^T)^T$ so we can now use `np.linalg.solve()`

# ---
# 
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

# Resources:
# 
# https://www.researchgate.net/post/Extended_Kalman_Filter_with_asynchronous_measurements
