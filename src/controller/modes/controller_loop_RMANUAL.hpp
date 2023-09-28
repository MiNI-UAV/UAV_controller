#pragma once
#include "../controller_loop.hpp"

class ControllerLoopRMANUAL: public ControllerLoop
{
public:
    ControllerLoopRMANUAL();

    void job(
        [[maybe_unused]] std::map<std::string,PID>& pids,
        Control& control,
        [[maybe_unused]] NS& navisys) override;
    void handleJoystick(Eigen::Vector4d joystick) override;
    std::string demandInfo() override;

protected:
    double demandedX;
    double demandedY;
};