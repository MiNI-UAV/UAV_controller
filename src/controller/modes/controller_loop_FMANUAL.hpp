#pragma once
#include "../controller_loop.hpp"

class ControllerLoopFMANUAL: public ControllerLoop
{
public:
    ControllerLoopFMANUAL();

    void job(
        [[maybe_unused]] std::map<std::string,std::unique_ptr<Controller>>& controllers,
        Control& control,
        [[maybe_unused]] NS& navisys) override;
        
    void handleJoystick(Eigen::VectorXd joystick) override;

private:
    std::atomic<double> demanded_P_rate = 0.0;
    std::atomic<double> demanded_Q_rate = 0.0;
    std::atomic<double> demanded_R_rate = 0.0;
    std::atomic<double> throttle = 0.0;
};