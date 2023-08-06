#pragma once
#include <Eigen/Dense>
#include <random>
#include <optional>
#include "environment.hpp"
#include "sensors.hpp"
#include "logger.hpp"
#include "timed_loop.hpp"

class AHRS
{
public:
    AHRS(Environment& env);
    ~AHRS();

    Eigen::Vector3d getOri();
    virtual Eigen::Vector3d getGyroBias();
    virtual Eigen::Matrix3d rot_bw() = 0;

    virtual void update() = 0;

protected:
    Eigen::Vector3d ori_est;
    std::mutex mtxOri;

    Environment& env;
    Logger logger;
};