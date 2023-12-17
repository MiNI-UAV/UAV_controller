#pragma once
#include "../controller_loop.hpp"

class ControllerLoopQPOS: public ControllerLoop
{
public:
    ControllerLoopQPOS();

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
    std::atomic<double> demandedX = 0.0;
    std::atomic<double> demandedY = 0.0;
    std::atomic<double> demandedZ = 0.0;
	std::atomic<double> demandedPsi = 0.0;
};