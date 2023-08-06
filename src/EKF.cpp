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

void EKF::predict(Eigen::Vector3d acc) {}

void EKF::updateBaro(double baro) {}

void EKF::updateGPS(Eigen::Vector3d pos) {}

void EKF::updateGPSVel(Eigen::Vector3d vel) {}
