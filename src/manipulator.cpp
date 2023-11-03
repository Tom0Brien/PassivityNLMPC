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
    double K = 1;
    Eigen::Matrix<double, num_states / 2, 1> q_d;
    q_d << 1, 1;
    optsolver.setObjectiveFunction([&](const Eigen::Matrix<double, pred_hor + 1, num_states>& x,
                                       const Eigen::Matrix<double, pred_hor + 1, num_output>&,
                                       const Eigen::Matrix<double, pred_hor + 1, num_inputs>& u,
                                       double) {
        double cost_val = 0;
        for (int i = 0; i < pred_hor + 1; i++) {
            // Extract the states
            Eigen::Matrix<double, num_states / 2, 1> q  = x.row(i).head(2);
            Eigen::Matrix<double, num_states / 2, 1> qd = x.row(i).tail(2);

            // Compute the mass matrix
            Eigen::Matrix<double, num_states / 2, num_states / 2> M = mass_matrix(model, q);

            // Define error
            Eigen::Matrix<double, num_states / 2, 1> e_q = q - q_d;
            Eigen::Matrix<double, 1, 1> cost             = 0.5 * (qd.transpose() * M * qd + e_q.transpose() * K * e_q);
            cost_val += cost(0, 0);
        }
        return cost_val;
    });

    // Define the Passivity constraint
    double rho = 0.1;
    optsolver.setIneqConFunction([&](Eigen::Matrix<double, ineq_c, 1>& in_con,
                                     const Eigen::Matrix<double, pred_hor + 1, num_states>& x,
                                     const Eigen::Matrix<double, pred_hor + 1, num_output>&,
                                     const Eigen::Matrix<double, pred_hor + 1, num_inputs>& u,
                                     const double&) {
        for (int i = 0; i < ineq_c; i++) {
            Eigen::Matrix<double, num_states / 2, 1> q  = x.row(i).head(2);
            Eigen::Matrix<double, num_states / 2, 1> qd = x.row(i).tail(2);
            Eigen::Matrix<double, num_inputs, 1> u_i    = u.row(i).transpose();
            Eigen::Matrix<double, 1, 1> constraint      = u_i.transpose() * qd + rho * qd.transpose() * qd;
            in_con(i)                                   = constraint(0, 0);
        }
    });

    // Simulate the system
    mpc::cvec<num_states> x, dx;
    x.resize(num_states);
    dx.resize(num_states);

    x(0) = 0.2;
    x(1) = 0.1;

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
    double tspan = 20;
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

    // Plot the results using python
    std::system("python ../src/plot_manipulator_results.py");

    // Animation
    std::system("python ../src/animate_manipulator_results.py");

    return 0;
}
