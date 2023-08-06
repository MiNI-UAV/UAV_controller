#pragma once
#include <Eigen/Dense>
#include "environment.hpp"
#include "sensors.hpp"
#include "timed_loop.hpp"

class EKF
{
public:
    Eigen::Vector3d getPos();
    Eigen::Vector3d getVel();

    void predict(Eigen::Vector3d acc);
    void updateBaro(double baro);
    void updateGPS(Eigen::Vector3d pos);
    void updateGPSVel(Eigen::Vector3d vel);
};