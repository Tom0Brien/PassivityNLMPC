import time
import pybullet_data
import pybullet as p
import numpy as np
import matplotlib.pyplot as plt

# Load the data from the file
data = np.loadtxt('nmpc_trajectory.dat', skiprows=1)

# Unpack the data
t, q1, q2, qd1, qd2, u1, u2, cost = data.T

# Connect to PyBullet
p.connect(p.GUI)

# Optionally, you can disable real-time simulation.
p.setRealTimeSimulation(0)

# Load the URDF(adjust the path to your URDF file)
robot_id = p.loadURDF("../urdfs/2_link.urdf", useFixedBase=True)

# Loop over your data and set the joint positions
for i in range(len(data)):
    # Set the joint position 1
    p.setJointMotorControl2(
        bodyIndex=robot_id,
        jointIndex=0,
        controlMode=p.POSITION_CONTROL,
        targetPosition=q1[i]
    )
    # Set the joint position 2
    p.setJointMotorControl2(
        bodyIndex=robot_id,
        jointIndex=1,
        controlMode=p.POSITION_CONTROL,
        targetPosition=q2[i]
    )

    # Optionally, sleep to control the visualization speed
    time.sleep(0.05)

    # You must manually update the renderings in PyBullet
    p.stepSimulation()

# Disconnect from PyBullet
p.disconnect()
