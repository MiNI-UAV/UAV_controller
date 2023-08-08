#include "EKF.hpp"
#include <Eigen/Dense>
#include <iostream>

EKF::EKF():
    logger("EKF.csv", "Time,PosX,PosY,PosZ,VelX,VelY,VelZ")
{
    x.setZero();

    CBaro << 0.0,0.0,1.0,0.0,0.0,0.0;
    CGPSPos.setZero();
    CGPSPos.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    CGPSVel.setZero();
    CGPSVel.block<3,3>(0,3) = Eigen::Matrix3d::Identity();

    Q.setIdentity();
    Q *= 0.001;
    Q(3,3) = 0.1;
    Q(6,6) = 0.1;
    RBaro = 0.0000000001;
    RGPSPos.setIdentity();
    RGPSPos *= 0.0000000001;
    RGPSVel.setIdentity();
    RGPSVel *= 0.0000000001;
    P = Q;
}

Eigen::Vector3d EKF::getPos() { return Eigen::Vector3d(0.0, 0.0, 0.0); }

Eigen::Vector3d EKF::getVel()
{
    return Eigen::Vector3d(0.0,0.0,0.0);
}

void EKF::predict(double time, Eigen::Vector3d acc) 
{
    static double last_update = 0.0;
    if(time == 0.0 && last_update == 0.0) 
    {
        last_update = time;
        return;
    }
    double T = time - last_update;

    Eigen::Matrix<double,6,6> A;
    Eigen::Matrix<double,6,3> B;
    A.setIdentity();
    B.setZero();
    A.block<3,3>(0,3) = T * Eigen::Matrix3d::Identity();
    B.block<3,3>(0,0) = (T*T/2.0) * Eigen::Matrix3d::Identity();
    B.block<3,3>(3,0) = T * Eigen::Matrix3d::Identity();

    x = A*x + B*acc;
    P = A*P*A.transpose() + Q;

    last_update = time;
}

void EKF::updateBaro(double time, double baro) 
{
    if(time == 0.0) return;
    auto K = P * CBaro.transpose() / (CBaro*P*CBaro.transpose() + RBaro);
    x = x + K*(baro - CBaro*x);
    P = (Eigen::Matrix<double,6,6>::Identity() - K*CBaro)*P;
}

void EKF::updateGPS(double time, Eigen::Vector3d pos) 
{
    if(time == 0.0) return;
    Eigen::Matrix3d inv_den = (CGPSPos*P*CGPSPos.transpose() + RGPSPos).inverse();
    auto K =  P * CGPSPos.transpose() * inv_den;
    x = x + K*(pos - CGPSPos*x);
    P = (Eigen::Matrix<double,6,6>::Identity() - K*CGPSPos)*P;
}

void EKF::updateGPSVel(double time, Eigen::Vector3d vel) 
{
    if(time == 0.0) return;
    Eigen::Matrix3d inv_den = (CGPSVel*P*CGPSVel.transpose() + RGPSVel).inverse();
    auto K =  P * CGPSVel.transpose() * inv_den;
    x = x + K*(vel - CGPSVel*x);
    P = (Eigen::Matrix<double,6,6>::Identity() - K*CGPSVel)*P;
}

void EKF::log(double time) 
{
    logger.log(time,{x});
}
