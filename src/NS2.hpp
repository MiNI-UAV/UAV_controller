#pragma once
#include <Eigen/Dense>
#include "environment.hpp"
#include "sensors.hpp"
#include "timed_loop.hpp"
#include "AHRS.hpp"
#include "EKF.hpp"

class NS2
{
public:

    NS2(Environment& env);
    ~NS2();
    //In world frame
    Eigen::Vector3d getPosition();
    Eigen::Vector3d getLinearVelocity();
    Eigen::Vector3d getOrientation();
    //In body frame
    Eigen::Vector3d getAngularVelocity();

private:
    Environment& env;
    std::unique_ptr<AHRS> ahrs;
    EKF ekf;

    std::thread loop_thread;
    TimedLoop loop;
    Status status;

    void job();
};