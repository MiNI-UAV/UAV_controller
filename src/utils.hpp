#pragma once
#include <Eigen/Dense>
#include <mutex>

template <typename T>
inline void safeSet(T& vec, T& new_val, std::mutex& mtx)
{
    std::lock_guard<std::mutex> guard(mtx);
    vec = new_val;
}

template <typename T>
inline T safeGet(T& vec, std::mutex& mtx)
{
    std::lock_guard<std::mutex> guard(mtx);
    return vec;
}

inline double circularError(double demanded, double val)
{
    double diff = demanded-val;
    if(diff > std::numbers::pi) return -2*std::numbers::pi + diff;
    if(diff < -std::numbers::pi) return +2*std::numbers::pi + diff;
    return diff;
}

inline double clampAngle(double angle)
{
    angle = std::fmod(angle + std::numbers::pi,2*std::numbers::pi);
    if (angle < 0)
        angle += 2*std::numbers::pi;
    return angle - std::numbers::pi;
}