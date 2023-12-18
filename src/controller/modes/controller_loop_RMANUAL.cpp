#include "controller_loop_RMANUAL.hpp"

ControllerLoopRMANUAL::ControllerLoopRMANUAL():
    ControllerLoop(ControllerMode::RMANUAL)
{
    required_controllers.assign({"H", "V"});
}

void ControllerLoopRMANUAL::job(
    [[maybe_unused]] std::map<std::string,std::unique_ptr<Controller>>& controllers,
    Control& control,
    [[maybe_unused]] NS& navisys
) 
{
    Eigen::Vector3d ori = navisys.getOrientation();
    Eigen::Vector3d angVel = navisys.getAngularVelocity();


    double est_roll = ori(0);

    double V_rate = controllers.at("V")->calc(demanded_V,angVel(1));
    double H_rate = controllers.at("H")->calc(demanded_H,angVel(2));

    if(std::abs(angVel(0)) < 3.0)
    {
        Eigen::VectorXd surf = applyMixerSurfaces(0.0, 0.0, 0.0, 0.0);
        control.sendSurface(surf);
        return;
    }
    double rot_pitch = V_rate * cos(est_roll) + H_rate * sin(est_roll);
    Eigen::VectorXd surf = applyMixerSurfaces(0.0, 0.0, rot_pitch, 0.0);
    control.sendSurface(surf);
}

void ControllerLoopRMANUAL::handleJoystick(Eigen::VectorXd joystick) 
{
    if(!checkJoystickLength(joystick,4)) return;
    demanded_H = joystick[1]*5.0;
    demanded_V = -joystick[2]*5.0;
}

std::string ControllerLoopRMANUAL::demandInfo() {
    std::stringstream ss;
    std::string s;
    ss.precision(3);
    ss << ControllerModeToString(_mode);
    return ss.str();
}
