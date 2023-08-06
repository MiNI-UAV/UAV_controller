#include "AHRS.hpp"
#include <Eigen/Dense>
#include <random>
#include "environment.hpp"
#include "sensors.hpp"
#include "logger.hpp"
#include "status.hpp"

AHRS::AHRS(Environment& env, int updatePeriodInMs):
    env{env},
    logger("ahrs.csv"),
    updatePeriodInMs{updatePeriodInMs}
{
    ori_est = Eigen::Vector3d(0.0,0.0,0.0);
    status = Status::running;
}

AHRS::~AHRS() 
{
    status = Status::exiting;
    std::cout << "AHRS exited." << std::endl;
}

Eigen::Vector3d AHRS::getOri()
{
    std::scoped_lock lck(mtxOri);
    return ori_est;
}

void AHRS::run()
{
    loop.emplace(updatePeriodInMs,[this](){update();},status);
    loopThread = std::thread([this]() {
        loop->go();
        });
}
