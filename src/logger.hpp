#pragma once
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <initializer_list>

#ifndef LOGGER_MASK
#define LOGGER_MASK 0
#endif

class Logger
{
public:
    Logger(std::string path, std::string fmt, uint8_t group = 0);
    ~Logger();

    void log(double time, std::initializer_list<Eigen::Vector3d> args);

private:
    std::ofstream file;
    const uint8_t group;
};

