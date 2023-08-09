#pragma once
#include <Eigen/Dense>
#include <random>
#include <atomic>
#include "../UAV_logger/logger.hpp"

class Environment;

template <class T>
class Sensor
{
public:
    Sensor(Environment& env, double sd, T bias,
        std::string path, std::string fmt, double refreshTime);

    virtual void update() = 0;
    bool shouldUpdate();
    inline T getReading() {
        ready = false;
        return value;
        };
    inline double getSd() {return dist.stddev();}
    inline bool isReady() {return ready.load();}

protected:
    Environment& env;
    T value;
    double refreshTime;
    double lastUpdate;
    std::atomic_bool ready;

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

class GPS : public Sensor<Eigen::Vector3d>
{
public:
    GPS(Environment& env, double sd);
    void update() override;
};

class GPSVel : public Sensor<Eigen::Vector3d>
{
public:
    GPSVel(Environment& env, double sd);
    void update() override;
};
