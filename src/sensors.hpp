#pragma once
#include <Eigen/Dense>
#include <random>
#include "environment.hpp"

template <class T>
class Sensor
{
public:
    Sensor(Environment& env, double sd,
        std::string path, std::string fmt);
        
    virtual void update() = 0;
    inline T getReading() {return value;};
    inline double getSd() {return dist.stddev();}

protected:
    Environment& env;
    T value;

    static std::mt19937 gen;
    std::normal_distribution<double> dist;
    
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