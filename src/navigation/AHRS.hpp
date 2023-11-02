#pragma once
#include <Eigen/Dense>
#include <random>
#include <optional>
#include "environment.hpp"
#include "sensors.hpp"

/// @brief Attitude and heading reference system
class AHRS
{
public:
    /// @brief Constructor
    /// @param env eference to environment, where AHRS works
    AHRS(Environment& env);

    /// @brief Deconstructor
    ~AHRS();

    /// @brief Returns estimatied orientation vector (roll, pitch, yaw)
    /// @return estimatied orientation
    Eigen::Vector3d getOri();


    /// @brief Returns estimatied gyroscope bias
    /// @return gyroscope bias
    virtual Eigen::Vector3d getGyroBias();

    /// @brief Returns rotation matrix from body to world frame
    /// @return rotation matrix
    virtual Eigen::Matrix3d rot_bw() = 0;

    virtual void update(Eigen::Vector3d gyro, Eigen::Vector3d acc, Eigen::Vector3d mag) = 0;

protected:
    Eigen::Vector3d ori_est;
    std::mutex mtxOri;

    Environment& env;
    Logger logger;
};