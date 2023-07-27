#pragma once
#include <Eigen/Dense>
#include <mutex>

inline void safeSet(Eigen::Vector3d& vec, Eigen::Vector3d& new_val, std::mutex& mtx)
{
    std::lock_guard<std::mutex> guard(mtx);
    vec = new_val;
}

inline Eigen::Vector3d safeGet(Eigen::Vector3d& vec, std::mutex& mtx)
{
    std::lock_guard<std::mutex> guard(mtx);
    return vec;
}