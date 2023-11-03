#include <fstream>
#include <iostream>
#include <mpc/NLMPC.hpp>
#include <tinyrobotics/dynamics.hpp>
#include <tinyrobotics/model.hpp>
#include <tinyrobotics/parser.hpp>

int main() {
    using namespace tinyrobotics;

    // Create a robot model with 5 joints
    auto model = import_urdf<double, 2>("../urdfs/2_link.urdf");

    // Display details of model
    model.show_details();

    // Define the source and target links
    const std::string source_link_name = "ground";
    const std::string target_link_name = "link_2";

    // Set some random joint configuration, velocity and torque
    Eigen::Matrix<double, 2, 1> q = Eigen::Matrix<double, 2, 1>::Zero();
    q << 1, 2;
    Eigen::Matrix<double, 2, 1> qd = Eigen::Matrix<double, 2, 1>::Zero();
    qd << 1, 2;
    Eigen::Matrix<double, 2, 1> tau = Eigen::Matrix<double, 2, 1>::Zero();
    tau << 1, 2;

    // Compute forward kinematics
    auto H = forward_kinematics(model, q, target_link_name);

    // Compute the forward dynamics
    auto qdd = forward_dynamics(model, q, qd, tau);
    std::cout << "qdd: " << qdd.transpose() << std::endl;

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
    optsolver.setObjectiveFunction([&](const mpc::mat<pred_hor + 1, num_states>& x,
                                       const mpc::mat<pred_hor + 1, num_output>&,
                                       const mpc::mat<pred_hor + 1, num_inputs>& u,
                                       double) { return x.array().square().sum() + u.array().square().sum(); });

    // Define the constraints
    optsolver.setIneqConFunction([&](mpc::cvec<ineq_c>& in_con,
                                     const mpc::mat<pred_hor + 1, num_states>&,
                                     const mpc::mat<pred_hor + 1, num_output>&,
                                     const mpc::mat<pred_hor + 1, num_inputs>& u,
                                     const double&) {
        for (int i = 0; i < ineq_c; i++) {
            in_con(i) = u(i, 0) - 0.5;
        }
    });

    mpc::cvec<num_states> x, dx;
    x.resize(num_states);
    dx.resize(num_states);

    x(0) = 0.5;
    x(1) = 0;

    auto r = optsolver.getLastResult();

    // Open a file to save the data
    std::ofstream data_file("nmpc_trajectory.dat");
    if (!data_file.is_open()) {
        std::cerr << "Unable to open file for writing the trajectory." << std::endl;
        return -1;
    }

    // Write the header for the data file
    data_file << "time q1 q2 qd1 qd2 u1 u2\n";
    double time  = 0;
    double tspan = 50;
    while (time < tspan) {
        r        = optsolver.step(x, r.cmd);
        auto seq = optsolver.getOptimalSequence();

        // Save the state and command to the file
        data_file << time << " " << x.transpose() << " " << r.cmd.transpose() << std::endl;

        stateEq(dx, x, r.cmd);
        x += dx * ts;
        time += ts;
    }

    // Close the file
    data_file.close();

    return 0;
}
