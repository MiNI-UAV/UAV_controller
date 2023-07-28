#include "logger.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <initializer_list>

Logger::Logger(std::string path, std::string fmt):
    file(path)
{
    file << fmt << std::endl;
}

Logger::~Logger() 
{
    file.close();
}

void Logger::log(double time, std::initializer_list<Eigen::Vector3d> args) {
    static Eigen::IOFormat commaFormat(3, Eigen::DontAlignCols," ",",");
    file << time;
    for(auto v: args)
    {
        file << ',' << v.format(commaFormat);
    }
    file << std::endl;
}
