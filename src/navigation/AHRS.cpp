#include "AHRS.hpp"
#include <Eigen/Dense>
#include <random>
#include "common.hpp"

AHRS::AHRS(Environment& env):
    env{env},
    logger("ahrs.csv")
{
    const UAVparams* params = UAVparams::getSingleton();
    ori_est = params->initialOrientation;
}

AHRS::~AHRS() 
{
    std::cout << "AHRS exited." << std::endl;
}

Eigen::Vector3d AHRS::getOri()
{
    std::scoped_lock lck(mtxOri);
    return ori_est;
}

Eigen::Vector3d AHRS::getGyroBias()
{
    return Eigen::Vector3d(0.0,0.0,0.0);
}
