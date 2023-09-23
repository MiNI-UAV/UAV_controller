#include "controller_loop_QACRO.hpp"

ControllerLoopQACRO::ControllerLoopQACRO():
    ControllerLoop(ControllerMode::QACRO)
{}

void ControllerLoopQACRO::job(
    State* state,
    std::map<std::string,PID>& pids,
    Control& control,
    NS& navisys
) 
{
    Eigen::Vector3d angVel = navisys.getAngularVelocity();

    double roll_rate = pids.at("Roll").calc(state->demandedP-angVel(0));
    double pitch_rate = pids.at("Pitch").calc(state->demandedQ-angVel(1));
    double yaw_rate = pids.at("Yaw").calc(state->demandedR-angVel(2));

    Eigen::VectorXd vec = applyMixerRotorsHover(state->throttle,roll_rate,pitch_rate,yaw_rate);
    control.sendSpeed(vec);
}

void ControllerLoopQACRO::handleJoystick(State* state, Eigen::Vector4d joystick) 
{
    state->throttle = joystick[1];
    state->demandedP = joystick[2]*5.0;
    state->demandedQ = -joystick[3]*5.0;
    state->demandedR = joystick[0]*3.0;
}

std::string ControllerLoopQACRO::demandInfo(State* state) {
    std::stringstream ss;
    std::string s;
    ss.precision(3);
    ss << std::fixed << ControllerModeToString(_mode) << ",";
    ss << state->demandedP << "," << state->demandedQ << "," << state->demandedR << "," << state->throttle;
    return ss.str();
}
