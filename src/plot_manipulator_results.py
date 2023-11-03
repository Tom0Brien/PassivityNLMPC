import numpy as np
import matplotlib.pyplot as plt

# Load the data from the file
data = np.loadtxt('nmpc_trajectory.dat', skiprows=1)


# Unpack the data, now including the cost
time, q1, q2, qd1, qd2, u1, u2, cost_values = data.T

# Create a figure and a set of subplots
fig, axs = plt.subplots(4, 1, figsize=(10, 20))  # 4 rows, 1 column

# Plotting the joint angles
axs[0].plot(time, q1, label='q1')
axs[0].plot(time, q2, label='q2')
axs[0].set_xlabel('Time [s]')
axs[0].set_ylabel('Joint Angle [rad]')
axs[0].legend()
axs[0].grid(True)

# Plotting the joint velocities
axs[1].plot(time, qd1, label='qd1')
axs[1].plot(time, qd2, label='qd2')
axs[1].set_xlabel('Time [s]')
axs[1].set_ylabel('Joint Velocity [rad/s]')
axs[1].legend()
axs[1].grid(True)

# Plotting the control inputs
axs[2].plot(time, u1, label='u1')
axs[2].plot(time, u2, label='u2')
axs[2].set_xlabel('Time [s]')
axs[2].set_ylabel('Control Input [Nm]')
axs[2].legend()
axs[2].grid(True)

# Plotting the cost function value
axs[3].plot(time, cost_values, label='Cost Function Value')
axs[3].set_xlabel('Time [s]')
axs[3].set_ylabel('Cost')
axs[3].legend()
axs[3].grid(True)

# Adjust layout to prevent overlapping
# plt.tight_layout()

# Show the combined plot
plt.show()
