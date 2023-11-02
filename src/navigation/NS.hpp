#pragma once
#include <Eigen/Dense>
#include "environment.hpp"
#include "sensors.hpp"
#include "AHRS.hpp"
#include "EKF.hpp"

/// @brief Navigation system
class NS
{
public:

    /// @brief Consturctor
    /// @param env reference to environment, that NS navigate through 
    NS(Environment& env);

    /// @brief Deconstructor
    ~NS();
    
    /// @brief Returns position estimated by NS
    /// @return position vector in world frame
    Eigen::Vector3d getPosition();

    /// @brief Returns linear velocity estimated by NS
    /// @return linear velocity vector in world frame
    Eigen::Vector3d getLinearVelocity();

    /// @brief Returns orientation estimated by NS
    /// @return orientation vector (RPY) in world frame
    Eigen::Vector3d getOrientation();

    /// @brief Returns rates estimated by NS
    /// @return angular velocity vector (roll rate, pitch rate, yaw rate) in body frame
    Eigen::Vector3d getAngularVelocity();

private:
    Environment& env;
    std::unique_ptr<AHRS> ahrs;
    std::unique_ptr<EKF> ekf;

    std::thread loop_thread;
    TimedLoop loop;
    Status status;

    void job();
    EKFParams calcParams();
};