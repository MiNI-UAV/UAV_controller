#include "controller_loop_RAUTOLAUNCH.hpp"

ControllerLoopRAUTOLAUNCH::ControllerLoopRAUTOLAUNCH():
    ControllerLoop(ControllerMode::RAUTOLAUNCH)
{
}

void ControllerLoopRAUTOLAUNCH::job(
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
}

