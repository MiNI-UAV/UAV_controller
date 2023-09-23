#pragma once
#include "../controller_loop.hpp"

class ControllerLoopQACRO: public ControllerLoop
{
public:
    ControllerLoopQACRO();

    void job(
        State* state,
        std::map<std::string,PID>& pids,
        Control& control,
        NS& navisys) override;
    void handleJoystick(State* state, Eigen::Vector4d joystick) override;
    std::string demandInfo(State* state) override;
};