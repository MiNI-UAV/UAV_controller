#include "logger.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <initializer_list>

bool shouldLog(uint8_t group)
{
    return LOGGER_MASK & (1 < group);
}

Logger::Logger(std::string path, std::string fmt, uint8_t group):
    file(path), group{group}
{
    file << fmt << std::endl;
}

Logger::~Logger() 
{
    file.close();
}

void Logger::log(double time, std::initializer_list<Eigen::Vector3d> args) {
    static Eigen::IOFormat commaFormat(3, Eigen::DontAlignCols," ",",");
    if(!shouldLog(group)) return;
    file << time;
    for(auto v: args)
    {
        file << ',' << v.format(commaFormat);
    }
    file << std::endl;
}
