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
    /// @brief Constructor
    /// @param ctx zero mq context
    /// @param uav_address address to state PUB socket that enviroment should listen
    Environment(zmq::context_t* ctx, std::string uav_address);

    /// @brief Deconstructor
    ~Environment();

    /// @brief Returns time of simulation
    /// @return simulation time
    double getTime();


    /// @brief Returns exact postion vector
    /// @return position vector in world frame
    Eigen::Vector3d getPosition();

#if USE_QUATERIONS
    /// @brief Returns exact orientation vector
    /// @return orientation vector in world frame
    Eigen::Vector4d getOrientation();
#else
    /// @brief Returns exact orientation vector
    /// @return orientation vector in world frame
    Eigen::Vector3d getOrientation();
#endif

    /// @brief Returns exact linear velocity vector
    /// @return linear velocity vector in world frame
    Eigen::Vector3d getWorldLinearVelocity();

    /// @brief Returns exact angular velocity vector
    /// @return linear angular vector in world frame
    Eigen::Vector3d getWorldAngularVelocity();

    /// @brief Returns exact linear velocity vector
    /// @return linear velocity vector in body frame
    Eigen::Vector3d getLinearVelocity();

    /// @brief Returns exact angular velocity vector
    /// @return angular velocities vector in body frame
    Eigen::Vector3d getAngularVelocity();

    /// @brief Returns exact linear acceleration vector
    /// @return linear acceleration vector in body frame
    Eigen::Vector3d getLinearAcceleration();

    /// @brief Returns exact angular acceleration vector
    /// @return angular acceleration vector in body frame
    Eigen::Vector3d getAngularAcceleraton();

    /// @brief Get rotation matrix from world to body frame
    /// @return rotation matrix
    Eigen::Matrix3d getRnb();

    /// @brief update all sensors
    void updateSensors();

    /// @brief map of sensors that measure values which is 3 element vector
    std::map<std::string,std::unique_ptr<Sensor<Eigen::Vector3d>>> sensorsVec3d;

    /// @brief map of sensors that measure single value
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