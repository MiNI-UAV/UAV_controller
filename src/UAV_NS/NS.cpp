#include "NS.hpp"
#include <Eigen/Dense>
#include <iostream>
#include "environment.hpp"
#include "sensors.hpp"
#include "AHRS.hpp"
#include "AHRS_EKF.hpp"
#include "AHRS_complementary.hpp"


NS::NS(Environment &env):
    env{env},
    loop(BASE_TIME_MS,[this](){job();},status)
{
    std::cout << "NS initializing..." << std::endl;
    Params* params = Params::getSingleton();
    status = Status::running;
    if(params->ahrs.type.compare("EKF") ==0)
    {
        ahrs = std::make_unique<AHRS_EKF>(env,params->ahrs.Q,params->ahrs.R);
    }
    if(params->ahrs.type.compare("Complementary") ==0)
    {
        ahrs = std::make_unique<AHRS_complementary>(env,params->ahrs.alpha);
    }
    if(ahrs.get() != nullptr) std::cout << "AHRS OK" << std::endl;
    ekf = std::make_unique<EKF>(calcParams());
    std::cout << "Parameters calculated" << std::endl;
    loop_thread = std::thread([this]() {loop.go();});
    std::cout << "NS initialized" << std::endl;
}

NS::~NS()
{
    status = Status::exiting;
    loop_thread.join();
}

Eigen::Vector3d NS::getPosition()
{
    return ekf->getPos();
}

Eigen::Vector3d NS::getLinearVelocity()
{
    return ekf->getVel();
}

Eigen::Vector3d NS::getOrientation()
{
    return ahrs->getOri();
}

Eigen::Vector3d NS::getAngularVelocity()
{
    return env.sensorsVec3d.at("gyroscope")->getReading() - ahrs->getGyroBias();
}

void NS::job() 
{
    env.updateSensors();
    double time = env.getTime();

    if(env.sensorsVec3d.at("accelerometer")->isReady() && env.sensorsVec3d.at("magnetometer")->isReady() && env.sensorsVec3d.at("gyroscope")->isReady())
    {
        auto acc = env.sensorsVec3d.at("accelerometer")->getReading();
        ahrs->update(env.sensorsVec3d.at("gyroscope")->getReading(), acc.normalized(), env.sensorsVec3d.at("magnetometer")->getReading().normalized());
        ekf->predict(time, ahrs->rot_bw()*acc - Accelerometer::g);
    }

    if(env.sensors.at("barometer")->isReady())
        ekf->updateBaro(time, env.sensors.at("barometer")->getReading());
    
    if(env.sensorsVec3d.at("GPS")->isReady())
        ekf->updateGPS(time, env.sensorsVec3d.at("GPS")->getReading());

    if(env.sensorsVec3d.at("GPSVel")->isReady())
        ekf->updateGPSVel(time, env.sensorsVec3d.at("GPSVel")->getReading());

    ekf->log(time);
}

EKFParams NS::calcParams()
{
    Params* params = Params::getSingleton();
    const double T = step_time/1000.0;
    const double predict_scaler = params->ekf.predictScaler;
    const double update_scaler = params->ekf.updateScaler;
    const double baro_scaler = params->ekf.baroScaler;
    const double z_extra_scaler = params->ekf.zScaler;

    EKFParams p;
    p.Q.setZero();
    p.Q.block<3,3>(0,0) = ((std::pow(T,4)/4.0)* std::pow(env.sensorsVec3d.at("gyroscope")->getSd(),2)) * predict_scaler * Eigen::Matrix3d::Identity();
    p.Q.block<3,3>(3,0) = ((std::pow(T,3)/2.0)* std::pow(env.sensorsVec3d.at("gyroscope")->getSd(),2)) * predict_scaler * Eigen::Matrix3d::Identity();
    p.Q.block<3,3>(0,3) = ((std::pow(T,3)/2.0)* std::pow(env.sensorsVec3d.at("gyroscope")->getSd(),2)) * predict_scaler * Eigen::Matrix3d::Identity();
    p.Q.block<3,3>(3,3) = ((std::pow(T,2)/1.0)* std::pow(env.sensorsVec3d.at("gyroscope")->getSd(),2)) * predict_scaler * Eigen::Matrix3d::Identity();
    p.Q(2,2) *= z_extra_scaler;
    p.Q(2,5) *= z_extra_scaler;
    p.Q(5,2) *= z_extra_scaler;
    p.Q(5,5) *= z_extra_scaler;
    p.RBaro = std::pow(env.sensors.at("barometer")->getSd(),2) * update_scaler * baro_scaler;
    p.RGPSPos.setIdentity();
    p.RGPSPos = Eigen::Matrix3d::Identity() * std::pow(env.sensorsVec3d.at("GPS")->getSd(),2) * update_scaler;
    p.RGPSVel.setIdentity();
    p.RGPSVel = Eigen::Matrix3d::Identity() *std::pow(env.sensorsVec3d.at("GPSVel")->getSd(),2) * update_scaler;
    p.P0.setZero();
    return p;
}
