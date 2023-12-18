#pragma once
#include "../controller_loop.hpp"

class ControllerLoopRANGLE: public ControllerLoop
{
public:
    ControllerLoopRANGLE();

    void job(
        [[maybe_unused]] std::map<std::string,std::unique_ptr<Controller>>& controllers,
        Control& control,
        [[maybe_unused]] NS& navisys) override;

    void handleJoystick(Eigen::VectorXd joystick) override;

    std::string demandInfo() override;

    void overridePositionAndSpeed(
        [[maybe_unused]] Eigen::Vector3d position,
        [[maybe_unused]] Eigen::Vector3d orientation,
        [[maybe_unused]] Eigen::Vector3d velocity
    ) override;

protected:
    std::atomic<double> demandedTheta = 0.0;
    std::atomic<double> demandedPsi = 0.0;

    static constexpr double angleLimit = std::numbers::pi/2.0;
};