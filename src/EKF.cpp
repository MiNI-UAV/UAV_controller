#include "EKF.hpp"
#include <Eigen/Dense>

Eigen::Vector3d EKF::getPos()
{
    return Eigen::Vector3d(0.0,0.0,0.0);
}

Eigen::Vector3d EKF::getVel()
{
    return Eigen::Vector3d(0.0,0.0,0.0);
}
