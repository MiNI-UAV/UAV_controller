#pragma once
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <initializer_list>

class Logger
{
public:
    Logger(std::string path, std::string fmt);
    ~Logger();

    void log(double time, std::initializer_list<Eigen::Vector3d> args);
private:
    std::ofstream file;
};

