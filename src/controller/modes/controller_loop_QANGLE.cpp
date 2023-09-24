#include "controller_loop_QANGLE.hpp"
#include "../../utils.hpp"

ControllerLoopQANGLE::ControllerLoopQANGLE():
    ControllerLoop(ControllerMode::QANGLE)
{
    required_pids.assign({"Roll", "Pitch", "Yaw", "W", "Z", "Fi", "Theta", "Psi"});
}

void ControllerLoopQANGLE::job(
    State* state,
    std::map<std::string,PID>& pids,
    Control& control,
    NS& navisys
) 
{
    Eigen::Vector3d pos = navisys.getPosition();
    Eigen::Vector3d vel = navisys.getLinearVelocity();
    Eigen::Vector3d ori = navisys.getOrientation();
    Eigen::Vector3d angVel = navisys.getAngularVelocity();

    double demandedW = pids.at("Z").calc(state->demandedZ - pos(2));
    double demandedP = pids.at("Fi").calc(circularError(state->demandedFi, ori(0)));
    double demandedQ = pids.at("Theta").calc(circularError(state->demandedTheta, ori(1)));
    double demandedR = pids.at("Psi").calc(circularError(state->demandedPsi, ori(2)));

    double climb_rate = pids.at("W").calc(demandedW-vel(2));
    double roll_rate = pids.at("Roll").calc(demandedP-angVel(0));
    double pitch_rate = pids.at("Pitch").calc(demandedQ-angVel(1));
    double yaw_rate = pids.at("Yaw").calc(demandedR-angVel(2));

    Eigen::VectorXd vec = applyMixerRotors(climb_rate,roll_rate,pitch_rate,yaw_rate);
    control.sendSpeed(vec);
}

void ControllerLoopQANGLE::handleJoystick(State* state, Eigen::Vector4d joystick) 
{
    constexpr double angleLimit = std::numbers::pi/5.0;
    state->demandedZ -= joystick[1]/10.0;
    state->demandedFi = joystick[2]*angleLimit;
    state->demandedTheta = -joystick[3]*angleLimit;
    state->demandedPsi = clampAngle(state->demandedPsi + joystick[0]/30.0);
}

std::string ControllerLoopQANGLE::demandInfo(State* state) {
    std::stringstream ss;
    std::string s;
    ss.precision(3);
    ss << std::fixed << ControllerModeToString(_mode) << ",";
    ss << state->demandedFi << "," << state->demandedTheta << "," << state->demandedPsi << "," << state->demandedZ;
    return ss.str();
}
