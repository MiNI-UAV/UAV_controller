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