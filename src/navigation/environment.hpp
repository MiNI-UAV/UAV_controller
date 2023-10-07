#pragma once
#include <zmq.hpp>
#include <thread>
#include <Eigen/Dense>
#include <mutex>
#include <vector>
#include <memory>
#include <atomic>
#include <map>
#include "sensors.hpp"
#include "common.hpp"
#include "../defines.hpp"

class Environment
{
public:
    Environment(zmq::context_t* ctx, std::string uav_address);
    ~Environment();

    double getTime();
    //In world frame
    Eigen::Vector3d getPosition();
#if USE_QUATERIONS
    Eigen::Vector4d getOrientation();
#else
    Eigen::Vector3d getOrientation();
#endif
    Eigen::Vector3d getWorldLinearVelocity();
    Eigen::Vector3d getWorldAngularVelocity();
    //In body frame
    Eigen::Vector3d getLinearVelocity();
    Eigen::Vector3d getAngularVelocity();
    Eigen::Vector3d getLinearAcceleration();
    Eigen::Vector3d getAngularAcceleraton();

    Eigen::Matrix3d getRnb();

    //Sensors
    void updateSensors();
    std::map<std::string,std::unique_ptr<Sensor<Eigen::Vector3d>>> sensorsVec3d;
    std::map<std::string,std::unique_ptr<Sensor<double>>> sensors;

private:
    std::atomic_bool run;

    std::atomic<double> time;
    Eigen::Vector3d position;
#if USE_QUATERIONS
    Eigen::Vector4d orientation;
#else
    Eigen::Vector3d orientation;
#endif
    Eigen::Vector3d worldLinearVelocity;
    Eigen::Vector3d worldAngularVelocity;
    Eigen::Vector3d linearVelocity;
    Eigen::Vector3d angularVelocity;
    Eigen::Vector3d linearAcceleration;
    Eigen::Vector3d angularAcceleration;
    Eigen::Matrix3d R_nb;
      
    std::mutex mtxPos;
    std::mutex mtxOri;
    std::mutex mtxWorldLinVel;
    std::mutex mtxWorldAngVel;
    std::mutex mtxLinVel;
    std::mutex mtxAngVel;
    std::mutex mtxLinAcc;
    std::mutex mtxAngAcc;
    std::mutex mtxRnb;

    zmq::socket_t time_sock;
    zmq::socket_t pos_sock;
    zmq::socket_t vel_sock;
    zmq::socket_t vel_world_sock;
    zmq::socket_t accel_sock;

    Logger logger;
    std::thread listener;
    void listenerJob();
};