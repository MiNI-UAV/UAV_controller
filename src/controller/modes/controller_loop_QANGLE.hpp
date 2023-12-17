#pragma once
#include "../controller_loop.hpp"

class ControllerLoopQANGLE: public ControllerLoop
{
public:
    ControllerLoopQANGLE();

    void job(
        std::map<std::string,std::unique_ptr<Controller>>& controllers,
        Control& control,
        NS& navisys) override;
        
    void handleJoystick(Eigen::VectorXd joystick) override;

    std::string demandInfo() override;

    void overridePositionAndSpeed(
        [[maybe_unused]] Eigen::Vector3d position,
        [[maybe_unused]] Eigen::Vector3d orientation,
        [[maybe_unused]] Eigen::Vector3d velocity
    ) override;

private:
    std::atomic<double> demandedZ = 0.0;
    std::atomic<double> demandedFi = 0.0;
    std::atomic<double> demandedTheta = 0.0;
    std::atomic<double> demandedPsi = 0.0;
};