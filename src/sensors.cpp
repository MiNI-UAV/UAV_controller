#include "sensors.hpp"
#include <Eigen/Dense>
#include <random>
#include "environment.hpp"

std::mt19937 Sensor::gen = std::mt19937(std::random_device()());

Sensor::Sensor(Environment &env, double sd,
    std::string path, std::string fmt):
    env{env}, dist(0.0,sd), logger(path,fmt,1)
{
    value = Eigen::Vector3d(0.0,0.0,0.0);
}

Eigen::Vector3d Sensor::getReading()
{
    return value;
}

double Sensor::error()
{
    return dist(gen);
}

Accelerometer::Accelerometer(Environment &env, double sd):
    Sensor(env,sd,"accelerometer.csv", "Time,AccX,AccY,AccZ"),
    g(0.0,0.0,9.81)
{}

void Accelerometer::update() 
{
    double time = env.getTime();
    auto rnb = env.getRnb();
    auto accel = env.getLinearAcceleration();

    value = accel + rnb*g + Eigen::Vector3d(error(),error(),error());
    logger.log(time,{value});
}

Gyroscope::Gyroscope(Environment &env, double sd):
    Sensor(env,sd,"gyroscope.csv", "Time,GyrX,GyrY,GyrZ")
{}

void Gyroscope::update() 
{
    double time = env.getTime();
    value = env.getAngularVelocity() + Eigen::Vector3d(error(),error(),error());
    logger.log(time,{value});
}

Magnetometer::Magnetometer(Environment &env, double sd):
    Sensor(env,sd,"magnetometer.csv", "Time,MagX,MagY,MagZ"),
    mag(60.0,0.0,0.0)
{}

void Magnetometer::update() 
{
    double time = env.getTime();
    auto rnb = env.getRnb();
    value = rnb*mag + Eigen::Vector3d(error(),error(),error());
    logger.log(time,{value});
}
