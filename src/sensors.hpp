#pragma once
#include <Eigen/Dense>
#include <random>
#include "environment.hpp"

class Sensor
{
public:
    Sensor(Environment& env, double sd,
        std::string path, std::string fmt);
    virtual void update() = 0;
    Eigen::Vector3d getReading();

protected:
    Environment& env;
    Eigen::Vector3d value;

    static std::mt19937 gen;
    std::normal_distribution<double> dist;
    double error();

    Logger logger;
};

class Accelerometer : public Sensor
{
public:
    Accelerometer(Environment& env, double sd);
    void update() override;
private:
    const Eigen::Vector3d g;
};

class Gyroscope : public Sensor
{
public:
    Gyroscope(Environment& env, double sd);
    void update() override;
};

class Magnetometer : public Sensor
{
public:
    Magnetometer(Environment& env, double sd);
    void update() override;
private:
    const Eigen::Vector3d mag;
};