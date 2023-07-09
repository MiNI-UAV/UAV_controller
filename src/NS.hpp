#pragma once
#include <zmq.hpp>
#include <thread>
#include <Eigen/Dense>
#include <mutex>
#include <vector>
#include <memory>
#include "sensor.hpp"

class Sensor;

class NS
{
    public:

        friend class Sensor;

        NS(zmq::context_t* ctx, std::string uav_address);
        ~NS() {run = false;};
        //In world frame
        Eigen::Vector3d getPosition();
        Eigen::Vector3d getOrientation();
        //In body frame
        Eigen::Vector3d getLinearVelocity();
        Eigen::Vector3d getAngularVelocity();
        Eigen::Vector3d getLinearAcceleration();
        Eigen::Vector3d getAngularAcceleraton();

        Eigen::Vector3d getWorldLinearVelocity();

    protected:
        bool run;
        std::vector<std::unique_ptr<Sensor>> sensors;

        Eigen::Vector3d position;
        Eigen::Vector3d orientation;
        Eigen::Vector3d linearVelocity;
        Eigen::Vector3d angularVelocity;
        Eigen::Vector3d linearAcceleration;
        Eigen::Vector3d angularAcceleration;
        
        
        std::mutex mtxPos;
        std::mutex mtxOri;
        std::mutex mtxLinVel;
        std::mutex mtxAngVel;
        std::mutex mtxLinAcc;
        std::mutex mtxAngAcc;

        void setPosition(Eigen::Vector3d newValue);
        void setOrientation(Eigen::Vector3d newValue);
        void setLinearVelocity(Eigen::Vector3d newValue);
        void setAngularVelocity(Eigen::Vector3d newValue);
        void setLinearAcceleration(Eigen::Vector3d newValue);
        void setAngularAcceleraton(Eigen::Vector3d newValue);
};