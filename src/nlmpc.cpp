#include <mpc/NLMPC.hpp>

#ifdef MPC_DYNAMIC
    #define MPC_DYNAMIC_TEST_NAME "Dynamic - "
    #define MPC_DYNAMIC_TEST_TAGS "[dynamic]"
constexpr bool DYNAMIC_ALLOC = true;
#else
    #define MPC_DYNAMIC_TEST_NAME "Static - "
    #define MPC_DYNAMIC_TEST_TAGS "[static]"
constexpr bool DYNAMIC_ALLOC = false;
#endif

constexpr int TVAR(int v) {
    int ret = -1;
    if (!DYNAMIC_ALLOC) {
        ret = v;
    }

    return ret;
}

#define MPC_TEST_NAME(name) MPC_DYNAMIC_TEST_NAME name
#define MPC_TEST_TAGS(tags) MPC_DYNAMIC_TEST_TAGS tags
int main() {

    constexpr int num_states = 2;
    constexpr int num_output = 2;
    constexpr int num_inputs = 1;
    constexpr int pred_hor   = 10;
    constexpr int ctrl_hor   = 5;
    constexpr int ineq_c     = pred_hor + 1;
    constexpr int eq_c       = 0;

    double ts = 0.1;

    mpc::NLMPC<num_states, num_inputs, num_output, pred_hor, ctrl_hor, ineq_c, eq_c> optsolver;

    optsolver.setLoggerLevel(mpc::Logger::log_level::NORMAL);
    optsolver.setContinuosTimeModel(ts);

    auto stateEq = [&](mpc::cvec<TVAR(num_states)>& dx,
                       const mpc::cvec<TVAR(num_states)>& x,
                       const mpc::cvec<TVAR(num_inputs)>& u) {
        dx(0) = ((1.0 - (x(1) * x(1))) * x(0)) - x(1) + u(0);
        dx(1) = x(0);
    };

    optsolver.setStateSpaceFunction([&](mpc::cvec<TVAR(num_states)>& dx,
                                        const mpc::cvec<TVAR(num_states)>& x,
                                        const mpc::cvec<TVAR(num_inputs)>& u,
                                        const unsigned int&) { stateEq(dx, x, u); });

    optsolver.setObjectiveFunction([&](const mpc::mat<TVAR(pred_hor + 1), TVAR(num_states)>& x,
                                       const mpc::mat<TVAR(pred_hor + 1), TVAR(num_output)>&,
                                       const mpc::mat<TVAR(pred_hor + 1), TVAR(num_inputs)>& u,
                                       double) { return x.array().square().sum() + u.array().square().sum(); });

    optsolver.setIneqConFunction([&](mpc::cvec<TVAR(ineq_c)>& in_con,
                                     const mpc::mat<TVAR(pred_hor + 1), TVAR(num_states)>&,
                                     const mpc::mat<TVAR(pred_hor + 1), TVAR(num_output)>&,
                                     const mpc::mat<TVAR(pred_hor + 1), TVAR(num_inputs)>& u,
                                     const double&) {
        for (int i = 0; i < ineq_c; i++) {
            in_con(i) = u(i, 0) - 0.5;
        }
    });

    mpc::cvec<TVAR(num_states)> modelX, modeldX;
    modelX.resize(num_states);
    modeldX.resize(num_states);

    modelX(0) = 0;
    modelX(1) = 1.0;

    auto r = optsolver.getLastResult();

    for (;;) {
        r        = optsolver.step(modelX, r.cmd);
        auto seq = optsolver.getOptimalSequence();
        (void) seq;
        stateEq(modeldX, modelX, r.cmd);
        modelX += modeldX * ts;
        if (std::fabs(modelX[0]) <= 1e-2 && std::fabs(modelX[1]) <= 1e-1) {
            break;
        }
    }

    return 0;
}