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
};