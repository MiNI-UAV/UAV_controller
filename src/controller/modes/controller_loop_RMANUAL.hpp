#pragma once
#include "../controller_loop.hpp"

class ControllerLoopRMANUAL: public ControllerLoop
{
public:
    ControllerLoopRMANUAL();

    void job(
        [[maybe_unused]] std::map<std::string,std::unique_ptr<Controller>>& controllers,
        Control& control,
        [[maybe_unused]] NS& navisys) override;
    void handleJoystick(Eigen::VectorXd joystick) override;
    std::string demandInfo() override;

protected:
    std::atomic<double> demanded_H = 0.0;
    std::atomic<double> demanded_V = 0.0;
};