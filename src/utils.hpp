#pragma once
#include <Eigen/Dense>
#include <mutex>

/// @brief Safe setter for T type value protected by mutex
/// @tparam T Type of variable
/// @param vec value to be set
/// @param new_val new value
/// @param mtx mutex
template <typename T>
inline void safeSet(T& vec, T& new_val, std::mutex& mtx)
{
    std::lock_guard<std::mutex> guard(mtx);
    vec = new_val;
}

/// @brief Safe getter for T type value protected by mutex
/// @tparam T Type of variable
/// @param vec value to be get
/// @param mtx mutex
/// @return value of vec
template <typename T>
inline T safeGet(T& vec, std::mutex& mtx)
{
    std::lock_guard<std::mutex> guard(mtx);
    return vec;
}

/// @brief Calculates error between demanded and actual angle. Finds shorter path.
/// For example if actual value is -0.9pi and demanded is 0.9pi error is equal -0.2pi
/// @param demanded demanded angle in radian
/// @param val actual angle in radian
/// @return angle error
inline double circularError(double demanded, double val)
{
    double diff = demanded-val;
    if(diff > std::numbers::pi) return -2*std::numbers::pi + diff;
    if(diff < -std::numbers::pi) return +2*std::numbers::pi + diff;
    return diff;
}

/// @brief Clamps angle given in radians to range <-pi,pi>
/// @param angle angle in radian
/// @return angle converted to range <-pi,pi>
inline double clampAngle(double angle)
{
    angle = std::fmod(angle + std::numbers::pi,2*std::numbers::pi);
    if (angle < 0)
        angle += 2*std::numbers::pi;
    return angle - std::numbers::pi;
}