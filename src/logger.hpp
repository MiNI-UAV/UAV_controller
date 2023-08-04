#pragma once
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <initializer_list>

#define LOGGER_MASK 5

class Logger
{
public:
    Logger(std::string path, std::string fmt = "", uint8_t group = 0);
    ~Logger();

    void setFmt(std::string fmt);
    void log(double time, std::initializer_list<Eigen::VectorXd> args);

private:
    std::ofstream file;
    const uint8_t group;
};

