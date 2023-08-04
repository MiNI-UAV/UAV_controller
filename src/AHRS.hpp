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
    AHRS(Environment& env, int updatePeriodInMs);
    ~AHRS();

    virtual void update() = 0;
    void run();

protected:
    Eigen::Vector3d ori_est;
    std::mutex mtxOri;

    Environment& env;
    Logger logger;

    Accelerometer acc;
    Gyroscope gyro;
    Magnetometer mag;

    Status status;
    const int updatePeriodInMs;
    std::optional<TimedLoop> loop;
    std::thread loopThread;
};