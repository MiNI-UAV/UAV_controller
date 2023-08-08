#include "NS2.hpp"
#include <Eigen/Dense>
#include <iostream>
#include "status.hpp"
#include "environment.hpp"
#include "sensors.hpp"
#include "timed_loop.hpp"
#include "AHRS.hpp"
#include "AHRS_EKF.hpp"
#include "AHRS_complementary.hpp"

NS2::NS2(Environment &env):
    env{env},
    loop(3,[this](){job();},status)
{
    status = Status::running;
    ahrs = std::make_unique<AHRS_complementary>(env,0.98);
    loop_thread = std::thread([this]() {loop.go();});
}

NS2::~NS2()
{
    status = Status::exiting;
    loop_thread.join();
}

Eigen::Vector3d NS2::getPosition()
{
    return ekf.getPos();
}

Eigen::Vector3d NS2::getLinearVelocity()
{
    return ekf.getVel();
}

Eigen::Vector3d NS2::getOrientation()
{
    return ahrs->getOri();
}

Eigen::Vector3d NS2::getAngularVelocity()
{
    return env.gyro.getReading() - ahrs->getGyroBias();
}

void NS2::job() 
{
    env.updateSensors();
    double time = env.getTime();

    if(env.acc.isReady() && env.mag.isReady() && env.gyro.isReady())
    {
        auto acc = env.acc.getReading();
        ahrs->update(env.gyro.getReading(), acc, env.mag.getReading());
        ekf.predict(time, ahrs->rot_bw()*acc - env.acc.g);
    }

    // if(env.baro.isReady())
    //     ekf.updateBaro(time, env.baro.getReading());
    
    if(env.gps.isReady())
        ekf.updateGPS(time, env.gps.getReading());

    if(env.gpsVel.isReady())
        ekf.updateGPSVel(time, env.gpsVel.getReading());

    ekf.log(time);
}
