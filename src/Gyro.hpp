#pragma once
#include <zmq.hpp>
#include <thread>
#include <Eigen/Dense>
#include <mutex>

class Gyro
{
    public:
        Gyro(zmq::context_t* ctx, std::string uav_address);
        ~Gyro();
        Eigen::Vector3d getAngularVel();
        void handleVelMsg(std::string msg);

    private:
        std::thread angularVelListener;
        Eigen::Vector3d angularVel;
        std::mutex mtxAngVel;

        void setAngularVel(Eigen::Vector3d newVel);
        Eigen::Vector3d gyroError(Eigen::Vector3d angularVel);
};