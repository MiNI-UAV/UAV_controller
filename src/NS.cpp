#include "NS.hpp"
#include <zmq.hpp>
#include <thread>
#include <Eigen/Dense>
#include <mutex>
#include <vector>
#include <memory>
#include "sensor.hpp"

Eigen::Vector3d safeGet(Eigen::Vector3d& vec, std::mutex& mtx)
{
    std::lock_guard<std::mutex> guard(mtx);
    return vec;
}

NS::NS(zmq::context_t *ctx, std::string uav_address)
{
    position.setZero();
    orientation.setZero();
    linearVelocity.setZero();
    angularVelocity.setZero();
    linearAcceleration.setZero();
    angularAcceleration.setZero();
    
    run = true;
    uav_address += "/state";
    sensors.push_back(std::move(std::make_unique<GNSS>(ctx,uav_address,*this,run,0.0)));
    sensors.push_back(std::move(std::make_unique<Gyroscope>(ctx,uav_address,*this,run,0.0)));
    sensors.push_back(std::move(std::make_unique<Accelerometer>(ctx,uav_address,*this,run,0.0,0.0)));
    sensors.push_back(std::move(std::make_unique<MagicLinearVelocitySensor>(ctx,uav_address,*this,run)));
    sensors.push_back(std::move(std::make_unique<MagicOrientationSensor>(ctx,uav_address,*this,run)));
}

Eigen::Vector3d NS::getPosition()
{
    return safeGet(position,mtxPos);
}

Eigen::Vector3d NS::getOrientation()
{
    return safeGet(orientation,mtxOri);
}

Eigen::Vector3d NS::getLinearVelocity()
{
    return safeGet(linearVelocity,mtxLinVel);
}

Eigen::Vector3d NS::getAngularVelocity()
{
    return safeGet(angularVelocity,mtxAngVel);
}

Eigen::Vector3d NS::getLinearAcceleration()
{
    return safeGet(linearAcceleration,mtxLinAcc);
}

Eigen::Vector3d NS::getAngularAcceleraton()
{
    return safeGet(angularAcceleration,mtxAngAcc);
}

Eigen::Matrix3d T(Eigen::Vector3d ori)
{
    Eigen::Matrix3d Tv;
    double fi = ori(0);
    double theta = ori(1);
    double psi = ori(2);

    Tv  << cos(theta)*cos(psi), sin(fi)*sin(theta)*cos(psi) - cos(fi)*sin(psi), cos(fi)*sin(theta)*cos(psi) + sin(fi)*sin(psi),
           cos(theta)*sin(psi), sin(fi)*sin(theta)*sin(psi) + cos(fi)*cos(psi), cos(fi)*sin(theta)*sin(psi) - sin(fi)*cos(psi),
           -sin(theta)        , sin(fi)*cos(theta)                            , cos(fi)*cos(theta);
    
    return Tv;
}

Eigen::Vector3d NS::getWorldLinearVelocity()
{
    return T(orientation)*linearVelocity;
}

void safeSet(Eigen::Vector3d& vec, std::mutex& mtx, Eigen::Vector3d& newValue)
{
    std::lock_guard<std::mutex> guard(mtx);
    vec = newValue;
}

void NS::setPosition(Eigen::Vector3d newValue)
{
    safeSet(position,mtxPos,newValue);
}

void NS::setOrientation(Eigen::Vector3d newValue) 
{
    safeSet(orientation,mtxOri,newValue);
}

void NS::setLinearVelocity(Eigen::Vector3d newValue) 
{
    safeSet(linearVelocity,mtxLinVel,newValue);
}

void NS::setAngularVelocity(Eigen::Vector3d newValue) 
{
    safeSet(angularVelocity,mtxAngVel,newValue);
}

void NS::setLinearAcceleration(Eigen::Vector3d newValue) 
{
    safeSet(linearAcceleration,mtxLinAcc,newValue);
}

void NS::setAngularAcceleraton(Eigen::Vector3d newValue) 
{
    safeSet(angularAcceleration,mtxAngAcc,newValue);
}
