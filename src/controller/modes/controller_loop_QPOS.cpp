#include "controller_loop_QPOS.hpp"
#include "../../utils.hpp"

ControllerLoopQPOS::ControllerLoopQPOS():
    ControllerLoop(ControllerMode::QPOS)
{
    required_pids.assign({"Roll", "Pitch", "Yaw", "W", "Z", "Fi",
        "Theta", "Psi", "X", "Y", "V", "U"});
}

void ControllerLoopQPOS::job(
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

    double demandedU = pids.at("X").calc(state->demandedX - pos(0));
    double demandedV = pids.at("Y").calc(state->demandedY - pos(1));
    
    double demandedFi_star = pids.at("V").calc(demandedV - vel(1));
    double demandedTheta_star = pids.at("U").calc(demandedU - vel(0));

    double PsiCos = std::cos(ori(2));
    double PsiSin = std::sin(ori(2));
    double demandedFi = demandedFi_star*PsiCos + demandedTheta_star*PsiSin;
    double demandedTheta = - demandedFi_star*PsiSin + demandedTheta_star*PsiCos;

    double demandedP = pids.at("Fi").calc(demandedFi - ori(0));
    double demandedQ = pids.at("Theta").calc(demandedTheta - ori(1));

    double demandedW = pids.at("Z").calc(state->demandedZ - pos(2));
    double demandedR = pids.at("Psi").calc(circularError(state->demandedPsi, ori(2)));

    double climb_rate = pids.at("W").calc(demandedW-vel(2));
    double roll_rate = pids.at("Roll").calc(demandedP-angVel(0));
    double pitch_rate = pids.at("Pitch").calc(demandedQ-angVel(1));
    double yaw_rate = pids.at("Yaw").calc(demandedR-angVel(2));

    Eigen::VectorXd vec = applyMixerRotors(climb_rate,roll_rate,pitch_rate,yaw_rate);
    control.sendSpeed(vec);
}

void ControllerLoopQPOS::handleJoystick(State* state, Eigen::Vector4d joystick) 
{
    constexpr double angleLimit = std::numbers::pi/5.0;
    state->demandedZ -= joystick[1]/10.0;
    state->demandedPsi = clampAngle(state->demandedPsi + joystick[0]/20.0);
    state->demandedX += ((joystick[3]*angleLimit)*std::cos(state->demandedPsi) - (joystick[2]*angleLimit)*std::sin(state->demandedPsi))/2.0;
    state->demandedY += ((joystick[3]*angleLimit)*std::sin(state->demandedPsi) + (joystick[2]*angleLimit)*std::cos(state->demandedPsi))/2.0;
}

std::string ControllerLoopQPOS::demandInfo(State* state) {
    std::stringstream ss;
    std::string s;
    ss.precision(3);
    ss << std::fixed << ControllerModeToString(_mode) << ",";
    ss << state->demandedX << "," << state->demandedY << "," << state->demandedZ << "," << state->demandedPsi;
    return ss.str();
}
