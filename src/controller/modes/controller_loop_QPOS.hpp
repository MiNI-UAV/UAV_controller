#pragma once
#include "../controller_loop.hpp"

class ControllerLoopQPOS: public ControllerLoop
{
public:
    ControllerLoopQPOS();

    void job(
        State* state,
        std::map<std::string,PID>& pids,
        Control& control,
        NS& navisys) override;
    void handleJoystick(State* state, Eigen::Vector4d joystick) override;
    std::string demandInfo(State* state) override;
};