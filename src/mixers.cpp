#include "mixers.hpp"
#include <Eigen/Dense>

Eigen::VectorXd controlMixer4(double verticalSpeed, double roll, double pitch, double yaw)
{
    const Eigen::Matrix4d mixer((
        Eigen::Matrix4d() << 1,  1,  1,  1,
                             1,  1, -1, -1,
                             1, -1, -1,  1,
                             1, -1,  1, -1).finished());
    Eigen::Vector4d u;
    u << verticalSpeed, roll, pitch, yaw;
    return mixer*u;
}