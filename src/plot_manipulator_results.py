import time
import pybullet_data
import pybullet as p
import numpy as np
import matplotlib.pyplot as plt

# Load the data from the file
data = np.loadtxt('nmpc_trajectory.dat', skiprows=1)

# Unpack the data
time, q1, q2, qd1, qd2, u1, u2 = data.T

# Plotting the joint angles
plt.figure()
plt.plot(time, q1, label='q1')
plt.plot(time, q2, label='q2')
plt.xlabel('Time [s]')
plt.ylabel('Joint Angle [rad]')
plt.title('Joint Angles over Time')
plt.legend()
plt.grid(True)

# Plotting the joint velocities
plt.figure()
plt.plot(time, qd1, label='qd1')
plt.plot(time, qd2, label='qd2')
plt.xlabel('Time [s]')
plt.ylabel('Joint Velocity [rad/s]')
plt.title('Joint Velocities over Time')
plt.legend()
plt.grid(True)

# Plotting the control inputs
plt.figure()
plt.plot(time, u1, label='u1')
plt.plot(time, u2, label='u2')
plt.xlabel('Time [s]')
plt.ylabel('Control Input [Nm]')
plt.title('Control Inputs over Time')
plt.legend()
plt.grid(True)

# Show all plots
plt.show()
