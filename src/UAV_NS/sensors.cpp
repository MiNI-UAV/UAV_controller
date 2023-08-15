#include "sensors.hpp"
#include <Eigen/Dense>
#include <random>
#include <limits>
#include "environment.hpp"
#include "common.hpp"


template <class T>
std::mt19937 Sensor<T>::gen = std::mt19937(std::random_device()());

template <class T>
Sensor<T>::Sensor(Environment &env, double sd, T bias,
    std::string path, std::string fmt, double refreshTime):
    env{env}, refreshTime{refreshTime}, dist(0.0,sd), bias{bias}, logger(path,fmt,1)
{
    lastUpdate = std::numeric_limits<double>::min();
    ready = false;
    value = T();
}

template <class T>
bool Sensor<T>::shouldUpdate()
{
    double time = env.getTime();
    if(time - lastUpdate > refreshTime)
    {
        lastUpdate = time;
        return true;
    }
    return false;
}

template <class T>
double Sensor<T>::error()
{
    return dist(gen);
}

const Eigen::Vector3d Accelerometer::g = Eigen::Vector3d(0.0,0.0,9.81);

Accelerometer::Accelerometer(Environment &env, double sd, Eigen::Vector3d bias, double refreshTime):
    Sensor<Eigen::Vector3d>(env, sd, bias, "accelerometer.csv", "Time,AccX,AccY,AccZ", refreshTime)
{}

void Accelerometer::update() 
{
    if(!shouldUpdate()) return;
    double time = env.getTime();
    auto rnb = env.getRnb();
    auto accel = env.getLinearAcceleration();

    value = accel + rnb*g + Eigen::Vector3d(error(),error(),error()) + bias;
    logger.log(time,{value});
    ready = true;
}

Gyroscope::Gyroscope(Environment &env, double sd, Eigen::Vector3d bias, double refreshTime):
    Sensor<Eigen::Vector3d>(env, sd, bias, "gyroscope.csv", "Time,GyrX,GyrY,GyrZ", refreshTime)
{}

void Gyroscope::update() 
{
    if(!shouldUpdate()) return;
    double time = env.getTime();
    value = env.getAngularVelocity() + Eigen::Vector3d(error(),error(),error()) + bias;
    logger.log(time,{value});
    ready = true;
}

const Eigen::Vector3d Magnetometer::mag = Eigen::Vector3d(60.0,0.0,0.0);

Magnetometer::Magnetometer(Environment &env, double sd, Eigen::Vector3d bias, double refreshTime):
    Sensor<Eigen::Vector3d>(env, sd, bias, "magnetometer.csv", "Time,MagX,MagY,MagZ", refreshTime)
{}

void Magnetometer::update() 
{
    if(!shouldUpdate()) return;
    double time = env.getTime();
    auto rnb = env.getRnb();
    value = rnb*mag + Eigen::Vector3d(error(),error(),error()) + bias;
    logger.log(time,{value});
    ready = true;
}

Barometer::Barometer(Environment &env, double sd, Eigen::Vector3d bias, double refreshTime):
    Sensor<double>(env, sd, bias[0], "barometer.csv", "Time,Height", refreshTime)
{}

void Barometer::update()
{
    if(!shouldUpdate()) return;
    double time = env.getTime();
    auto pos = env.getPosition();
    value = pos(2) + error();
    logger.log(time,{value});
    ready = true;
}

GPS::GPS(Environment &env, double sd, Eigen::Vector3d bias, double refreshTime):
    Sensor<Eigen::Vector3d>(env, sd, bias, "GPS.csv", "Time,PosX,PosY,PosZ", refreshTime)
{}

void GPS::update() 
{
    if(!shouldUpdate()) return;
    double time = env.getTime();
    auto pos = env.getPosition();
    value = pos + Eigen::Vector3d(error(),error(),error()) + bias;;
    logger.log(time,{value});
    ready = true;
}

GPSVel::GPSVel(Environment &env, double sd, Eigen::Vector3d bias, double refreshTime):
    Sensor<Eigen::Vector3d>(env, sd, bias, "GPSVel.csv", "Time,VelX,VelY,VelZ", refreshTime)
{}

void GPSVel::update()
{
    if(!shouldUpdate()) return;
    double time = env.getTime();
    auto vel = env.getWorldLinearVelocity();
    value = vel + Eigen::Vector3d(error(),error(),error()) + bias;;
    logger.log(time,{value});
    ready = true;
}
