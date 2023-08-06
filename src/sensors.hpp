#pragma once
#include <Eigen/Dense>
#include <random>
#include "logger.hpp"

class Environment;

template <class T>
class Sensor
{
public:
    Sensor(Environment& env, double sd, T bias,
        std::string path, std::string fmt, double refreshTime);

    virtual void update() = 0;
    bool shouldUpdate();
    inline T getReading() {return value;};
    inline double getSd() {return dist.stddev();}

protected:
    Environment& env;
    T value;
    double refreshTime;
    double lastUpdate;

    static std::mt19937 gen;
    std::normal_distribution<double> dist;
    T bias;
    
    double error();

    Logger logger;
};

class Accelerometer : public Sensor<Eigen::Vector3d>
{
public:
    Accelerometer(Environment& env, double sd);
    void update() override;
private:
    const Eigen::Vector3d g;
};

class Gyroscope : public Sensor<Eigen::Vector3d>
{
public:
    Gyroscope(Environment& env, double sd);
    void update() override;
};

class Magnetometer : public Sensor<Eigen::Vector3d>
{
public:
    Magnetometer(Environment& env, double sd);
    void update() override;
private:
    const Eigen::Vector3d mag;
};

class Barometer : public Sensor<double>
{
public:
    Barometer(Environment& env, double sd);
    void update() override;
};