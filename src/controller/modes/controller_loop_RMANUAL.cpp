#include "controller_loop_RMANUAL.hpp"

ControllerLoopRMANUAL::ControllerLoopRMANUAL():
    ControllerLoop(ControllerMode::RMANUAL)
{
}

void ControllerLoopRMANUAL::job(
    [[maybe_unused]] std::map<std::string,PID>& pids,
    Control& control,
    [[maybe_unused]] NS& navisys
) 
{
    static bool running = false;
    if(!running)
    {
        control.startJet(0);
        running = true;
    }

    control.sendHinge('j',0,0,demandedX);
    control.sendHinge('j',0,1,demandedY);
}

void ControllerLoopRMANUAL::handleJoystick(Eigen::VectorXd joystick) 
{
    demandedX = joystick[2]*0.1;
    demandedY = -joystick[1]*0.1;
}

std::string ControllerLoopRMANUAL::demandInfo() {
    std::stringstream ss;
    std::string s;
    ss.precision(3);
    ss << ControllerModeToString(_mode);
    return ss.str();
}
