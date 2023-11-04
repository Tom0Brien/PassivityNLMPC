#include <fstream>
#include <iostream>
#include <mpc/NLMPC.hpp>
#include <tinyrobotics/dynamics.hpp>
#include <tinyrobotics/math.hpp>
#include <tinyrobotics/model.hpp>
#include <tinyrobotics/parser.hpp>

int main() {
    using namespace tinyrobotics;

    // Create a robot model with 2 joints
    auto model = import_urdf<double, 2>("../urdfs/2_link.urdf");

    // Dimensions
    constexpr int num_states = 4;
    constexpr int num_output = 2;
    constexpr int num_inputs = 2;
    constexpr int pred_hor   = 10;
    constexpr int ctrl_hor   = 10;
    constexpr int ineq_c     = pred_hor + 1;
    constexpr int eq_c       = 0;

    double ts = 0.1;

    mpc::NLMPC<num_states, num_inputs, num_output, pred_hor, ctrl_hor, ineq_c, eq_c> optsolver;

    // Define the state space function
    auto stateEq = [&](Eigen::Matrix<double, num_states, 1>& dx,
                       const Eigen::Matrix<double, num_states, 1>& x,
                       const Eigen::Matrix<double, num_inputs, 1>& u) {
        Eigen::Matrix<double, num_states / 2, 1> q   = x.head(2);
        Eigen::Matrix<double, num_states / 2, 1> qd  = x.tail(2);
        Eigen::Matrix<double, num_states / 2, 1> qdd = forward_dynamics(model, q, qd, u);
        dx.head(2)                                   = qd;
        dx.tail(2)                                   = qdd;
    };
    optsolver.setContinuosTimeModel(ts);
    optsolver.setStateSpaceFunction([&](Eigen::Matrix<double, num_states, 1>& dx,
                                        const Eigen::Matrix<double, num_states, 1>& x,
                                        const Eigen::Matrix<double, num_inputs, 1>& u,
                                        const unsigned int&) { stateEq(dx, x, u); });

    // Define the objective function
    double K = 10;
    Eigen::Matrix<double, num_states / 2, 1> q_d;
    q_d << -0.785398, 0;
    Eigen::Transform<double, 3, Eigen::Isometry> H_desired = forward_kinematics(model, q_d, std::string("link_2"));
    std::cout << "Desired pose: " << std::endl;
    std::cout << H_desired.matrix() << std::endl;

    optsolver.setObjectiveFunction([&](const Eigen::Matrix<double, pred_hor + 1, num_states>& x,
                                       const Eigen::Matrix<double, pred_hor + 1, num_output>&,
                                       const Eigen::Matrix<double, pred_hor + 1, num_inputs>& u,
                                       double) {
        double cost = 0;
        for (int i = 0; i < pred_hor + 1; i++) {
            // Extract the states
            Eigen::Matrix<double, num_states / 2, 1> q  = x.row(i).head(2);
            Eigen::Matrix<double, num_states / 2, 1> qd = x.row(i).tail(2);

            // Compute the kinetic energy
            double kinetic_energy_i = kinetic_energy(model, q, qd);

            // Define error
            Eigen::Transform<double, 3, Eigen::Isometry> H_i = forward_kinematics(model, q, std::string("link_2"));
            Eigen::Matrix<double, 6, 1> e                    = homogeneous_error(H_i, H_desired);

            // Add to the cost
            cost += 0.5 * (kinetic_energy_i + e.transpose() * K * e);
        }
        return cost;
    });

    // Simulate the system
    mpc::cvec<num_states> x, dx;
    x.resize(num_states);
    dx.resize(num_states);

    x(0) = 0.2;
    x(1) = 0.1;

    auto r = optsolver.getLastResult();

    std::ofstream data_file("nmpc_trajectory.dat");
    if (!data_file.is_open()) {
        std::cerr << "Unable to open file for writing the trajectory." << std::endl;
        return -1;
    }

    data_file << "time q1 q2 qd1 qd2 u1 u2 cost\n";
    double time  = 0;
    double tspan = 20;
    while (time < tspan) {
        r        = optsolver.step(x, r.cmd);
        auto seq = optsolver.getOptimalSequence();

        // Save the state and command to the file
        data_file << time << " " << x.transpose() << " " << r.cmd.transpose() << " " << r.cost << std::endl;

        stateEq(dx, x, r.cmd);
        x += dx * ts;
        time += ts;
    }

    // Close the file
    data_file.close();

    // Plot the results using python
    std::system("python ../src/plot_manipulator_results.py");

    // Animation
    std::system("python ../src/animate_manipulator_results.py");

    return 0;
}
