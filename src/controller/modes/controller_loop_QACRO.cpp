#include "controller_loop_QACRO.hpp"

ControllerLoopQACRO::ControllerLoopQACRO():
    ControllerLoop(ControllerMode::QACRO)
{
    required_pids.assign({"Roll", "Pitch", "Yaw"});
}

void ControllerLoopQACRO::job(
    std::map<std::string,PID>& pids,
    Control& control,
    NS& navisys
) 
{
    Eigen::Vector3d angVel = navisys.getAngularVelocity();

    double roll_rate = pids.at("Roll").calc(demandedP-angVel(0));
    double pitch_rate = pids.at("Pitch").calc(demandedQ-angVel(1));
    double yaw_rate = pids.at("Yaw").calc(demandedR-angVel(2));

    Eigen::VectorXd vec = applyMixerRotorsHover(throttle,roll_rate,pitch_rate,yaw_rate);
    control.sendSpeed(vec);
}

void ControllerLoopQACRO::handleJoystick(Eigen::VectorXd joystick) 
{
    throttle = joystick[0];
    demandedP = joystick[1]*5.0;
    demandedQ = -joystick[2]*5.0;
    demandedR = joystick[3]*3.0;
}

std::string ControllerLoopQACRO::demandInfo() {
    std::stringstream ss;
    std::string s;
    ss.precision(3);
    ss << std::fixed << ControllerModeToString(_mode) << ",";
    ss << demandedP << "," << demandedQ << "," << demandedR << "," << throttle;
    return ss.str();
}
