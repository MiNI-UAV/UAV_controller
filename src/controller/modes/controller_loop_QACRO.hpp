#pragma once
#include "../controller_loop.hpp"

class ControllerLoopQACRO: public ControllerLoop
{
public:
    ControllerLoopQACRO();

    void job(
        std::map<std::string,PID>& pids,
        Control& control,
        NS& navisys) override;
        
    void handleJoystick(Eigen::VectorXd joystick) override;

    std::string demandInfo() override;

private:
    std::atomic<double> demandedP = 0.0;
    std::atomic<double> demandedQ = 0.0;
    std::atomic<double> demandedR = 0.0;
    std::atomic<double> throttle = 0.0;
};