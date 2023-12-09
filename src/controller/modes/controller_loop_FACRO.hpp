#pragma once
#include "../controller_loop.hpp"

class ControllerLoopFACRO: public ControllerLoop
{
public:
    ControllerLoopFACRO();

    void job(
        std::map<std::string,std::unique_ptr<Controller>>& controllers,
        Control& control,
        NS& navisys) override;
        
    void handleJoystick(Eigen::VectorXd joystick) override;

    std::string demandInfo() override;

private:
    std::atomic<double> demanded_P = 0.0;
    std::atomic<double> demanded_Q = 0.0;
    std::atomic<double> demanded_R = 0.0;
    std::atomic<double> throttle = 0.0;
};