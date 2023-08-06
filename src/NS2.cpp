#include "NS2.hpp"
#include <Eigen/Dense>
#include "environment.hpp"
#include "sensors.hpp"
#include "timed_loop.hpp"
#include "AHRS.hpp"
#include "AHRS_EKF.hpp"

NS2::NS2(Environment &env):
    env{env},
    loop(3,[this](){job();},status)
{
    ahrs = std::make_unique<AHRS_EKF>(env);
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

    if(env.acc.isReady() && env.mag.isReady() && env.gyro.isReady())
    {
        auto acc = env.acc.getReading();
        ahrs->update(env.gyro.getReading(), acc, env.mag.getReading());
        ekf.predict(acc);
    }

    if(env.baro.isReady())
        ekf.updateBaro(env.baro.getReading());
    
    if(env.gps.isReady())
        ekf.updateGPS(env.gps.getReading());

    if(env.gpsVel.isReady())
        ekf.updateGPSVel(env.gpsVel.getReading());

}
