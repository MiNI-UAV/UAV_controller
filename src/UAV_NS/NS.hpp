#pragma once
#include <Eigen/Dense>
#include "environment.hpp"
#include "sensors.hpp"
#include "AHRS.hpp"
#include "EKF.hpp"
#include "../UAV_common/timed_loop.hpp"


#define BASE_TIME_MS 3

class NS
{
public:

    NS(Environment& env);
    ~NS();
    //In world frame
    Eigen::Vector3d getPosition();
    Eigen::Vector3d getLinearVelocity();
    Eigen::Vector3d getOrientation();
    //In body frame
    Eigen::Vector3d getAngularVelocity();

private:
    Environment& env;
    std::unique_ptr<AHRS> ahrs;
    std::unique_ptr<EKF> ekf;

    std::thread loop_thread;
    TimedLoop loop;
    Status status;

    void job();
    EKFParams calcParams();
};