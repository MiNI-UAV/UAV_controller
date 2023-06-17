#include "mixers.hpp"
#include <Eigen/Dense>

Eigen::VectorXd controlMixer4(double climb_rate, double roll_rate , double pitch_rate, double yaw_rate, double maxSpeed)
{
    const Eigen::Matrix4d mixer((
        Eigen::Matrix4d() << 1,  1,  1,  1,
                             1,  1, -1, -1,
                             1, -1, -1,  1,
                             1, -1,  1, -1).finished());
    Eigen::Vector4d u;
    u << climb_rate, roll_rate, pitch_rate, yaw_rate;
    Eigen::VectorXd res = mixer*u;
    return res.cwiseMax(0.0).cwiseMin(maxSpeed);
}

Eigen::VectorXd controlMixer(Eigen::MatrixX4d mixer,double  climb_rate, double roll_rate , double pitch_rate, double yaw_rate, double maxSpeed)
{
    Eigen::Vector4d u;
    u << climb_rate, roll_rate, pitch_rate, yaw_rate;
    Eigen::VectorXd res = mixer*u;
    return res.cwiseMax(0.0).cwiseMin(maxSpeed);
}