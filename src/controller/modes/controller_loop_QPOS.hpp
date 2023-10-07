#pragma once
#include "../controller_loop.hpp"

class ControllerLoopQPOS: public ControllerLoop
{
public:
    ControllerLoopQPOS();

    void job(
        std::map<std::string,PID>& pids,
        Control& control,
        NS& navisys) override;

    void handleJoystick(Eigen::VectorXd joystick) override;

    std::string demandInfo() override;

    void overridePosition(
        Eigen::Vector3d position,
        Eigen::Vector3d orientation
    ) override;

private:
    std::atomic<double> demandedX = 0.0;
    std::atomic<double> demandedY = 0.0;
    std::atomic<double> demandedZ = 0.0;
	std::atomic<double> demandedPsi = 0.0;
};