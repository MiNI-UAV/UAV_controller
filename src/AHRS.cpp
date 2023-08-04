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
    acc(env,0.0),
    gyro(env,0.0),
    mag(env, 0.0),
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

void AHRS::run()
{
    loop.emplace(updatePeriodInMs,[this](){update();},status);
    loopThread = std::thread([this]() {
        loop->go();
        });
}