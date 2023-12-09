#include "controller_loop_FMANUAL.hpp"

ControllerLoopFMANUAL::ControllerLoopFMANUAL():
    ControllerLoop(ControllerMode::FMANUAL)
{
    required_controllers.clear();
}

void ControllerLoopFMANUAL::job(
    [[maybe_unused]] std::map<std::string,std::unique_ptr<Controller>>& controllers,
    Control& control,
    [[maybe_unused]] NS& navisys
) 
{

    Eigen::VectorXd vec = applyMixerRotorsHover(throttle,demanded_P_rate,demanded_Q_rate,demanded_R_rate);
    control.sendSpeed(vec);
    Eigen::VectorXd surf = applyMixerSurfaces(throttle,demanded_P_rate,demanded_Q_rate,demanded_R_rate);
    control.sendSurface(surf);
}

void ControllerLoopFMANUAL::handleJoystick(Eigen::VectorXd joystick) 
{
    if(!checkJoystickLength(joystick,4)) return;
    throttle = joystick[0];
    demanded_P_rate = joystick[1];
    demanded_Q_rate = -joystick[2];
    demanded_R_rate = joystick[3];
}
