#pragma once
#include "../controller_loop.hpp"

class ControllerLoopRMANUAL: public ControllerLoop
{
public:
    ControllerLoopRMANUAL();

    void job(
        [[maybe_unused]] State* state,
        [[maybe_unused]] std::map<std::string,PID>& pids,
        Control& control,
        [[maybe_unused]] NS& navisys) override;
    void handleJoystick([[maybe_unused]] State* state, Eigen::Vector4d joystick) override;
    std::string demandInfo([[maybe_unused]] State* state) override;

protected:
    double demandedX;
    double demandedY;
};