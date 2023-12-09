#pragma once
#include "../controller_loop.hpp"

class ControllerLoopFANGLE: public ControllerLoop
{
public:
    ControllerLoopFANGLE();

    void job(
        std::map<std::string,std::unique_ptr<Controller>>& controllers,
        Control& control,
        NS& navisys) override;
        
    void handleJoystick(Eigen::VectorXd joystick) override;

    std::string demandInfo() override;

    void overridePosition(
        [[maybe_unused]] Eigen::Vector3d position,
        Eigen::Vector3d orientation
    ) override;

    static constexpr double angleLimit = std::numbers::pi/5.0;

private:
    std::atomic<double> demandedVx = 0.0;
    std::atomic<double> demandedFi = 0.0;
    std::atomic<double> demandedTheta = 0.0;
    std::atomic<double> demandedPsi = 0.0;
    std::atomic<double> demanded_R = 0.0;
};