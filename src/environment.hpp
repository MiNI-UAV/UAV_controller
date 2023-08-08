#pragma once
#include <zmq.hpp>
#include <thread>
#include <Eigen/Dense>
#include <mutex>
#include <vector>
#include <memory>
#include <atomic>
#include "logger.hpp"
#include "sensors.hpp"

class Environment
{
public:
    Environment(zmq::context_t* ctx, std::string uav_address);
    ~Environment();

    double getTime();
    //In world frame
    Eigen::Vector3d getPosition();
    Eigen::Vector3d getOrientation();
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
    Accelerometer acc;
    Gyroscope gyro;
    Magnetometer mag;
    Barometer baro;
    GPS gps;
    GPSVel gpsVel;


private:
    std::atomic_bool run;

    std::atomic<double> time;
    Eigen::Vector3d position;
    Eigen::Vector3d orientation;
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