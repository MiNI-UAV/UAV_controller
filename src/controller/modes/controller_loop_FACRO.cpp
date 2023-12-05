#include "controller_loop_FACRO.hpp"
#include "../../utils.hpp"

ControllerLoopFACRO::ControllerLoopFACRO():
    ControllerLoop(ControllerMode::FACRO)
{
    required_pids.assign({"Roll", "Pitch", "Yaw"});
}

void ControllerLoopFACRO::job(
    [[maybe_unused]] std::map<std::string,PID>& pids,
    Control& control,
    [[maybe_unused]] NS& navisys
) 
{

    Eigen::Vector3d angVel = navisys.getAngularVelocity();

    double roll_rate = pids.at("Roll").calc(demanded_P-angVel(0));
    double pitch_rate = pids.at("Pitch").calc(demanded_Q-angVel(1));
    double yaw_rate = pids.at("Yaw").calc(demanded_R-angVel(2));

    Eigen::VectorXd vec = applyMixerRotorsHover(throttle,roll_rate,pitch_rate,yaw_rate);
    control.sendSpeed(vec);
    Eigen::VectorXd surf = applyMixerSurfaces(throttle,roll_rate,pitch_rate,yaw_rate);
    control.sendSurface(surf);
}

void ControllerLoopFACRO::handleJoystick(Eigen::VectorXd joystick) 
{
    if(!checkJoystickLength(joystick,4)) return;
    throttle   = joystick[0];
    demanded_P = joystick[1]*5.0;
    demanded_Q = -joystick[2]*5.0;
    demanded_R = joystick[3]*3.0;
}

std::string ControllerLoopFACRO::demandInfo() {
    std::stringstream ss;
    std::string s;
    ss.precision(3);
    ss << std::fixed << ControllerModeToString(_mode) << ",";
    ss << demanded_P << "," << demanded_Q << "," << demanded_R;
    return ss.str();
}
