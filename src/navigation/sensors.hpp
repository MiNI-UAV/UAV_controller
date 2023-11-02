#pragma once
#include <Eigen/Dense>
#include <random>
#include <atomic>
#include "common.hpp"

class Environment;

/// @brief Sensors base class
/// @tparam T type of data read by sensor
template <class T>
class Sensor
{
public:
    /// @brief Constructor
    /// @param env reference to environment sensor measures
    /// @param sd standard deviation of reading
    /// @param bias reading bias
    /// @param path path where sensor logs are saved
    /// @param fmt header of log file
    /// @param refreshTime sample period
    Sensor(Environment& env, double sd, T bias,
        std::string path, std::string fmt, double refreshTime);

    /// @brief Update sensor state. Measured value is updated if sensor is ready for next read.
    virtual void update() = 0;

    /// @brief Returns recent measure
    /// @return sensor measure
    inline T getReading() {
        ready = false;
        return value;
        };

    /// @brief Returns standard deviation
    /// @return standard deviation
    inline double getSd() {return dist.stddev();}

    
    /// @brief Checks if sensor is ready
    /// @return true if sensor is ready
    inline bool isReady() {return ready.load();}

protected:
    /// @brief Checks if sensor should measure next value
    /// @return true if sensor is ready for next measure
    bool shouldUpdate();

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

/// @brief Representation of accelerometer
class Accelerometer : public Sensor<Eigen::Vector3d>
{
public:
    Accelerometer(Environment& env, double sd, Eigen::Vector3d bias, double refreshTime);
    void update() override;
    static const Eigen::Vector3d g;
};

/// @brief Representation of gyroscope
class Gyroscope : public Sensor<Eigen::Vector3d>
{
public:
    Gyroscope(Environment& env, double sd, Eigen::Vector3d bias, double refreshTime);
    void update() override;
};

/// @brief Representation of magnetometer
class Magnetometer : public Sensor<Eigen::Vector3d>
{
public:
    Magnetometer(Environment& env, double sd, Eigen::Vector3d bias, double refreshTime);
    void update() override;
    static const Eigen::Vector3d mag;
};

/// @brief Representation of barometer
class Barometer : public Sensor<double>
{
public:
    Barometer(Environment& env, double sd, Eigen::Vector3d bias, double refreshTime);
    void update() override;
};

/// @brief Representation of GPS position measure
class GPS : public Sensor<Eigen::Vector3d>
{
public:
    GPS(Environment& env, double sd, Eigen::Vector3d bias, double refreshTime);
    void update() override;
};

/// @brief Representation of GPS velocity measure
class GPSVel : public Sensor<Eigen::Vector3d>
{
public:
    GPSVel(Environment& env, double sd, Eigen::Vector3d bias, double refreshTime);
    void update() override;
};
