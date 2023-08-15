#pragma once
#include <Eigen/Dense>
#include "environment.hpp"
#include "sensors.hpp"

struct EKFParams
{
    Eigen::Matrix<double,6,6> P0;
    Eigen::Matrix<double,6,6> Q;
    double RBaro;
    Eigen::Matrix3d RGPSPos;
    Eigen::Matrix3d RGPSVel;
};

class EKF
{
public:
    EKF(EKFParams params);
    Eigen::Vector3d getPos();
    Eigen::Vector3d getVel();

    void predict(double time, Eigen::Vector3d acc);
    void updateBaro(double time, double baro);
    void updateGPS(double time, Eigen::Vector3d pos);
    void updateGPSVel(double time, Eigen::Vector3d vel);
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