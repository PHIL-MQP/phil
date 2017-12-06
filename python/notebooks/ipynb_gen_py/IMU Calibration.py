
# coding: utf-8

# # How to do the calibration

# ## Code setup

# In[1]:

import matplotlib.pyplot as plot
from scipy import misc
import numpy as np
import csv
import sys
import os


# # The Algorithm

#  - The first step is to let the IMU sit at rest for $T_{init}$ seconds. This interval is derived from the Allen Variance metnhod.
#  - Also during this initial time interval, measure the average gyro readings in each axis: $\textbf{b}^g$
#  - Let the IMU sit at some orientation for 4 seconds, then move it to some other orientation. Repeat this between 36 and 50 times.
#  - Iterate over each gyroscope readings and subtract the bias $\text{b}^g$
#  - Compute the magnitude of the variance $\varsigma_{init}$ for the accelerometer data over the $T_{init}$. To do this, you iterate over each accelerometer reading during $T_{init}$ and compute the sample variance. This is the average of the variance for each point: $var(a) = (a-\mathbb{E}[a])^2$. Do this for each axis.

# #### A quick code example of this part:

# In[2]:

x = [1.1, 1.2, 0.9, 0.9, 1.3, 0.8]
expected_value = np.mean(x)
print("expected_value: ", expected_value)
sample_variance = np.mean((x - expected_value)**2)
print("sample variance:", sample_variance)


#  - Iterate over a bunch of thresholds and compute how good each one is at calibrating the data. This is what the loop in Algorithm 1 refers to.
#    - Set the threshold you're testing to $threshold = i * \varsigma_{init}$
#    - Compute the regions of static/dynamic using that threshold
#      - Iterate over each window of $t_{wait}$ size in the collected data and compute the sample variance (same as above) and compare to the threshold
#      - If it's greater, the you classify as dynamics, otherwise as static
#      - The output will be like the black line in Figure 3
#    - Do the LM optimization (magic call the Eigen).
#      - For this LM, we need to provide a functor that has `int operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const` and `int df(const Eigen::VectorXf &x, Eigen::MatrixXf &fjac) const`. For `operator()` it should fill `fvec` with the values of the equation $e_k={\lVert g\rVert}^2 - {\lVert T^aK^a(a_k^S+b^a)\rVert}^2$, where ${\lVert g\rVert}^2 = 9.8$. For `df`, you fill `fjac` with each error term $\frac{\partial e_1}{\partial \theta^{acc}}, \dots, \frac{\partial e_k}{\partial \theta^{acc}}$.
#       - Compute the residuals for the final optimized parameters
#       - Save (residuals, params, threshold, intervals) if they're the best ones so far
#  - Calibrate the accelerometer $a^o = T^aK^a(a^S+b^a)$
#  - Calibrate the gyroscope using another LM. This time in `int operator()` you fill `fvec` with $\lVert(u_{a,k} - u_{g,k})\rVert$ and also the partials into `fjac`.

# ## Setting up the Jacobians
# 
# Look in `docs/IMU_calibration.pdf`

# ## Evaluating the accuacy of numerical derivatives

# In[41]:

def func(theta_acc):
    T = np.array([[1, -theta_acc[0], theta_acc[1]],[0, 1, -theta_acc[2]],[0,0,1]])
    K = np.array([[theta_acc[3],0,0],[0,theta_acc[4],0],[0,0,theta_acc[5]]])
    b = np.array([[theta_acc[6]],[theta_acc[7]],[theta_acc[8]]])
    return 1 - np.linalg.norm(T@K@(a_i + b))**2

def five_point_approximation(func, theta_acc, h):
    N = theta_acc.shape[1]
    partials = np.ndarray((N,1))
    for partial_idx in range(N):
        hs = np.array([-2*h, -h, h, 2*h])
        nodes = np.repeat(theta_acc, 4, axis=0)
        nodes[:,partial_idx] += hs
        five_point_approx = 1/(12*h)*(func(nodes[0]) - 8*func(nodes[1]) + 8*func(nodes[2]) - func(nodes[3]))
        partials[partial_idx] = five_point_approx
    return partials


# theta_acc = [a_yz, a_zy, a_zx, s_x_a, s_y_a, s_z_a, b_x_a, b_y_a, b_z_a]
theta_acc = np.array([[0.1, 0, 0.1, 1, 1, 1, 0, 0.1, 0]], dtype=np.float32)
a_i = np.array([[0.2], [0.7], [0.1]])
h = 0.1

print("five-point approximations")
print(five_point_approximation(func, theta_acc, h))


# hs = np.array([-2*h, -h, h, 2*h])
# nodes = np.repeat(theta_acc, 4, axis=0)
# nodes[:,0] += hsReferences:
# 
#  - https://www.math.uwaterloo.ca/~hwolkowi/matrixcookbook.pdf
#  - http://eigen.tuxfamily.org/bz_attachmentbase/attachment.cgi?id=395
#  - https://medium.com/@sarvagya.vaish/levenberg-marquardt-optimization-part-1-981f5777b1d7
#  - https://medium.com/@sarvagya.vaish/levenberg-marquardt-optimization-part-2-5a71f7db27a0
#  - https://github.com/SarvagyaVaish/Eigen-Levenberg-Marquardt-Optimization/blob/master/main.cpp

# In[4]:

data_filename = "recorded_sensor_data/imu_calibration_11_14_20-00-00/imu_calibration_data_11_14.csv"
reader = csv.reader(open(data_filename, 'r'))

next(reader) # skip header
data = []
for row in reader:
    data.append([float(x) for x in row])
data = np.array(data)


# ## iterate over the Tinit period to compute the gyro biases

# In[5]:

Tinit = 4
samples_per_second = 100
Tinit_idx = 4 * samples_per_second
init_data = data[:Tinit_idx]
gyro_biases = np.mean(init_data, axis=0)[3:6]
print("gyro_biases:", gyro_biases)
sample_variance = np.var(init_data)

