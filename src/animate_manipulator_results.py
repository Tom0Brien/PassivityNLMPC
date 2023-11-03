import time
import pybullet_data
import pybullet as p
import numpy as np
import matplotlib.pyplot as plt

# Load the data from the file
data = np.loadtxt('nmpc_trajectory.dat', skiprows=1)

# Unpack the data
t, q1, q2, qd1, qd2, u1, u2 = data.T

# Connect to PyBullet
p.connect(p.GUI)

# Optionally, you can disable real-time simulation.
p.setRealTimeSimulation(0)

# Load the URDF(adjust the path to your URDF file)
robot_id = p.loadURDF("../urdfs/2_link.urdf", useFixedBase=True)

# Loop over your data and set the joint positions
for i in range(len(data)):
    # Extract the joint positions from your data
    joint_positions = data[i, 1:2]  # Assuming the first column is time

    print(joint_positions)
    # Iterate over each joint and set its position
    for joint_index in range(len(joint_positions)):
        # Set the joint position
        p.setJointMotorControl2(
            bodyIndex=robot_id,
            jointIndex=joint_index,
            controlMode=p.POSITION_CONTROL,
            targetPosition=joint_positions[joint_index]
        )

    # Optionally, sleep to control the visualization speed
    time.sleep(0.05)

    # You must manually update the renderings in PyBullet
    p.stepSimulation()

# Disconnect from PyBullet
p.disconnect()
