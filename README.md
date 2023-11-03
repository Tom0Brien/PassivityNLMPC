# Passivity Based Nonlinear Model Predictive Control for Robotic Manipulator

This repository contains a C++ implementation of Passivity Based Nonlinear Model Predictive Control (NMPC) designed for controlling a robotic manipulator with two joints. The implementation includes an objective function that minimizes a combination of kinetic energy and error to a desired state, and a passivity constraint to ensure stable control behavior. This is inpired by MATLAB example [3].

## Dependencies

- [tinyrobotics](https://github.com/Tom0Brien/tinyrobotics): A hypothetical library used for handling the robot model, dynamics, and parsing URDF files.
- [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page): A C++ template library for linear algebra.
- [libmpc++](https://github.com/nicolapiccinelli/libmpc): Model Predictive Control Library.

## Passivity Based Nonlinear Model Predictive
Passivity Based Nonlinear Model Predictive is an MPC control scheme where a passivity-based constraint is used to obtain a nonlinear model predictive control scheme with guaranteed closed loop stability for any, possible arbitrarily small, prediction horizon.

The dynamics of robotic manipulator can be expressed as:
$M(q) \ddot{q}+C(q, \dot{q}) \dot{q}+G(q)=\tau$

Here, $q, \dot{q}$, and $\ddot{q}$ are vectors that represent the joint angles, velocities, and accelerations. The control input vector is the torque $\tau$
- $M(q)$ is the manipulator inertia matrix.
- $C(q, \dot{q})$ is the Coriolis matrix.
- $G(q)$ is the gravity vector.

The control objective is to select torque $\tau$ such that the joint angles $q$ track a desired reference $q_d$. To enforce closed-loop stability, the controller includes a passivity constraint [2].

### Passivity Constraint
To define the passivity constraint, first we define the tracking error vector as the difference between the joint angles and the desired reference angles.


$e_q=q-q_d$

To achieve good tracking performance, we define the storage function as 


$V=\frac{1}{2}\left(\dot{q}^T M(q) \dot{q}+e_q^T K e_q\right)$, 

where $K>0$. Taking the derivative of $V$ we obtain the relationship $\dot{V}=u^T \dot{q}$, where
$u=\tau-G(q)+\mathrm{Ke}_q$
Therefore, the system is passive from $u$ to $\dot{q}$.

To enforce closed-loop stability, we define the passivity constraint as follows [2].


$u^T \dot{q} \leq-\rho \dot{q}^T \dot{q} \text { with } \rho>0 \text {. }$

See `src/manipulator.cpp` for code a example and results below.

### Results for 2 link manipulator

![Results](https://github.com/Tom0Brien/PassivityNLMPC/assets/41043317/7d0f0ac5-4c7f-414d-b222-3f5dcaf9a17f)

As depicted in the figure above, the cost function is strictly decreasing, a property inforced via the passivity constraint.

## References

[1] Hatanaka, Takeshi, Nikhil Chopra, Masayuki Fujita, and Mark W. Spong. Passivity-Based Control and Estimation in Networked Robotics. Communications and Control Engineering. Cham: Springer International Publishing, 2015. https://doi.org/10.1007/978-3-319-15171-7.

[2] Raff, Tobias, Christian Ebenbauer, and Frank Allgöwer. “Nonlinear Model Predictive Control: A Passivity-Based Approach.” In Assessment and Future Directions of Nonlinear Model Predictive Control, edited by Rolf Findeisen, Frank Allgöwer, and Lorenz T. Biegler, 358:151–62. Berlin, Heidelberg: Springer Berlin Heidelberg, 2007. https://doi.org/10.1007/978-3-540-72699-9_12.

[3] https://au.mathworks.com/help/mpc/ug/control-of-robot-manipulator-using-passivity-based-nonlinear-mpc.html




