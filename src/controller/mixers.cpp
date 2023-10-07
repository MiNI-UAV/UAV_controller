#include "mixers.hpp"
#include <Eigen/Dense>
#include "common.hpp"

Eigen::VectorXd applyMixerRotors(double climb_rate, double roll_rate , double pitch_rate, double yaw_rate)
{
    static const UAVparams* params = UAVparams::getSingleton();
    static const auto rotorMatrix = params->rotorMixer;
    static const auto rotorMaxSpeed = params->getRotorMaxSpeeds();

    Eigen::Vector4d u;
    u << climb_rate, roll_rate, pitch_rate, yaw_rate;
    Eigen::VectorXd res = rotorMatrix*u;
    return res.cwiseMax(0.0).cwiseMin(rotorMaxSpeed);
}

Eigen::VectorXd applyMixerRotorsHover(double throttle, double roll_rate, double pitch_rate, double yaw_rate)
{
    static const UAVparams* params = UAVparams::getSingleton();
    static const auto rotorMatrix = params->rotorMixer;
    static const auto rotorMaxSpeed = params->getRotorMaxSpeeds();
    static const auto rotorHoverSpeed = params->getRotorHoverSpeeds();

    Eigen::Vector4d u;
    u << 0.0, roll_rate, pitch_rate, yaw_rate;
    Eigen::VectorXd res = rotorMatrix*u;
    res+= (throttle + 1.0)*rotorHoverSpeed; 
    return res.cwiseMax(0.0).cwiseMin(rotorMaxSpeed);
}

Eigen::VectorXd applyMixerSurfaces(double throttle, double roll_rate, double pitch_rate, double yaw_rate)
{
    static const UAVparams* params = UAVparams::getSingleton();
    static const auto surfaceMatrix = params->surfaceMixer;

    Eigen::Vector4d u;
    u << throttle, roll_rate, pitch_rate, yaw_rate;
    Eigen::VectorXd res = surfaceMatrix*u; 
    return res;
}
