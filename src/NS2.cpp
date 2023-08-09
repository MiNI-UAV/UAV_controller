#include "NS2.hpp"
#include <Eigen/Dense>
#include <iostream>
#include "status.hpp"
#include "environment.hpp"
#include "sensors.hpp"
#include "timed_loop.hpp"
#include "AHRS.hpp"
#include "AHRS_EKF.hpp"
#include "AHRS_complementary.hpp"


NS2::NS2(Environment &env):
    env{env},
    loop(BASE_TIME_MS,[this](){job();},status)
{
    status = Status::running;
    ahrs = std::make_unique<AHRS_complementary>(env,0.98);
    ekf = std::make_unique<EKF>(calcParams());
    loop_thread = std::thread([this]() {loop.go();});
    std::cout << "NS initialized" << std::endl;
}

NS2::~NS2()
{
    status = Status::exiting;
    loop_thread.join();
}

Eigen::Vector3d NS2::getPosition()
{
    return ekf->getPos();
}

Eigen::Vector3d NS2::getLinearVelocity()
{
    return ekf->getVel();
}

Eigen::Vector3d NS2::getOrientation()
{
    return ahrs->getOri();
}

Eigen::Vector3d NS2::getAngularVelocity()
{
    return env.gyro.getReading() - ahrs->getGyroBias();
}

void NS2::job() 
{
    env.updateSensors();
    double time = env.getTime();

    if(env.acc.isReady() && env.mag.isReady() && env.gyro.isReady())
    {
        auto acc = env.acc.getReading();
        ahrs->update(env.gyro.getReading(), acc, env.mag.getReading());
        ekf->predict(time, ahrs->rot_bw()*acc - env.acc.g);
    }

    // if(env.baro.isReady())
    //     ekf.updateBaro(time, env.baro.getReading());
    
    if(env.gps.isReady())
        ekf->updateGPS(time, env.gps.getReading());

    if(env.gpsVel.isReady())
        ekf->updateGPSVel(time, env.gpsVel.getReading());

    ekf->log(time);
}

EKFParams NS2::calcParams()
{
    constexpr double T = BASE_TIME_MS/1000.0;
    constexpr double predict_scaler = 1e1;
    constexpr double update_scaler = 1e-4;
    constexpr double baro_scaler = 1e-4;
    constexpr double z_extra_scaler = 1e3;


    EKFParams p;
    p.Q.setZero();
    p.Q.block<3,3>(0,0) = ((std::pow(T,4)/4.0)* std::pow(env.gyro.getSd(),2)) * predict_scaler * Eigen::Matrix3d::Identity();
    p.Q.block<3,3>(3,0) = ((std::pow(T,3)/2.0)* std::pow(env.gyro.getSd(),2)) * predict_scaler * Eigen::Matrix3d::Identity();
    p.Q.block<3,3>(0,3) = ((std::pow(T,3)/2.0)* std::pow(env.gyro.getSd(),2)) * predict_scaler * Eigen::Matrix3d::Identity();
    p.Q.block<3,3>(3,3) = ((std::pow(T,2)/1.0)* std::pow(env.gyro.getSd(),2)) * predict_scaler * Eigen::Matrix3d::Identity();
    p.Q(2,2) *= z_extra_scaler;
    p.Q(2,5) *= z_extra_scaler;
    p.Q(5,2) *= z_extra_scaler;
    p.Q(5,5) *= z_extra_scaler;
    p.RBaro = std::pow(env.baro.getSd(),2) * update_scaler * baro_scaler;
    p.RGPSPos.setIdentity();
    p.RGPSPos = Eigen::Matrix3d::Identity() * std::pow(env.gps.getSd(),2) * update_scaler;
    p.RGPSVel.setIdentity();
    p.RGPSVel = Eigen::Matrix3d::Identity() *std::pow(env.gpsVel.getSd(),2) * update_scaler;
    p.P0.setZero();
    return p;
}
