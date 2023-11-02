#pragma once
#include <Eigen/Dense>
#include "environment.hpp"
#include "sensors.hpp"


/// @brief EK filer parameters
struct EKFParams
{
    Eigen::Matrix<double,6,6> P0;
    Eigen::Matrix<double,6,6> Q;
    double RBaro;
    Eigen::Matrix3d RGPSPos;
    Eigen::Matrix3d RGPSVel;
};

/// @brief Extended Kalman Filter
class EKF
{
public:

    /// @brief Constructor
    /// @param params filter parameters
    EKF(EKFParams params);


    /// @brief Returns estimated position vector
    /// @return position vector in world frame
    Eigen::Vector3d getPos();

    /// @brief Returns estimated velocity vector
    /// @return velocity vector in world frame
    Eigen::Vector3d getVel();

    /// @brief Predict phase. Integration of accelerometer measures.
    /// @param time simulation time
    /// @param acc accelerometer measure
    void predict(double time, Eigen::Vector3d acc);

    /// @brief Update phase. Height correction
    /// @param time simulation time
    /// @param baro barometer measure
    void updateBaro(double time, double baro);

    /// @brief Update phase. Position correction
    /// @param time simulation time
    /// @param baro GPS location measure
    void updateGPS(double time, Eigen::Vector3d pos);

    /// @brief Update phase. Velocity correction
    /// @param time simulation time
    /// @param baro GPS velocity measure
    void updateGPSVel(double time, Eigen::Vector3d vel);


    /// @brief Log filter state
    /// @param time simulation time
    void log(double time);

private:
    Logger logger;
    std::mutex mtx;
    Eigen::Vector<double,6> x;
    Eigen::Matrix<double,6,6> P;

    Eigen::Matrix<double,1,6> CBaro;
    Eigen::Matrix<double,3,6> CGPSPos;
    Eigen::Matrix<double,3,6> CGPSVel;

    const EKFParams params;
};