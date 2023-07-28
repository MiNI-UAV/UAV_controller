#include "environment.hpp"
#include <zmq.hpp>
#include <thread>
#include <Eigen/Dense>
#include <mutex>
#include <vector>
#include <memory>
#include <iostream>
#include <initializer_list>
#include "utils.hpp"
#include "logger.hpp"

void connectConflateSocket(zmq::socket_t& sock, std::string address, std::string topic)
{
    sock.set(zmq::sockopt::conflate,1);
    sock.set(zmq::sockopt::subscribe, topic);
    sock.connect(address);
}

Environment::Environment(zmq::context_t *ctx, std::string uav_address):
    time_sock(*ctx,zmq::socket_type::sub),
    pos_sock(*ctx,zmq::socket_type::sub),
    vel_sock(*ctx,zmq::socket_type::sub),
    accel_sock(*ctx,zmq::socket_type::sub),
    logger("env.csv", "time")
{
    uav_address += "/state";
    connectConflateSocket(time_sock, uav_address, "t:");
    connectConflateSocket(pos_sock, uav_address, "pos:");
    connectConflateSocket(vel_sock, uav_address, "vb:");
    connectConflateSocket(accel_sock, uav_address, "ab:");
    run.store(true,std::memory_order_relaxed);
    listener = std::thread(&Environment::listenerJob, this);
}

Environment::~Environment()
{
    run.store(false,std::memory_order_relaxed);
    listener.join();

    time_sock.close();
    pos_sock.close();
    vel_sock.close();
    accel_sock.close();
}

void recvVectors(zmq::socket_t& sock, int skip, Eigen::Vector3d& vec1, Eigen::Vector3d& vec2)
{
    zmq::message_t msg;
    if(!sock.recv(msg, zmq::recv_flags::none))
    {
        std::cerr << " listener recv error" << std::endl;
    }
    std::string msg_str =  std::string(static_cast<char*>(msg.data()), msg.size());
    std::istringstream f(msg_str.substr(skip));
    std::string s;
    int i; 
    for (i = 0; i < 6; i++)
    {
        if(!getline(f, s, ','))
        {
            std::cerr << "Invalid msg" << std::endl;
            break;
        }
        if(i < 3)
        {
            vec1(i) = std::stod(s);
        }
        else
        {
            vec2(i-3) = std::stod(s);
        }
    }
    if(i != 6)
    {
        std::cerr << "Invalid msg" << std::endl;
        return;
    }
}

void Environment::listenerJob() 
{
    double msg_time;
    Eigen::Vector3d msg_position;
    Eigen::Vector3d msg_orientation;
    Eigen::Vector3d msg_linearVelocity;
    Eigen::Vector3d msg_angularVelocity;
    Eigen::Vector3d msg_linearAcceleration;
    Eigen::Vector3d msg_angularAcceleration;

    while(run.load())
    {
        zmq::message_t msg;
        if(!time_sock.recv(msg, zmq::recv_flags::none))
        {
            std::cerr << " listener recv error" << std::endl;
        }
        std::string msg_str =  std::string(static_cast<char*>(msg.data()), msg.size());
        msg_time = std::stod(msg_str.substr(2));
        recvVectors(pos_sock,4,msg_position,msg_orientation);
        recvVectors(vel_sock,3,msg_linearVelocity,msg_angularVelocity);
        recvVectors(accel_sock,3,msg_linearAcceleration,msg_angularAcceleration);

        time.store(msg_time,std::memory_order_consume);
        safeSet(position,msg_position,mtxPos);
        safeSet(orientation,msg_orientation,mtxOri);
        safeSet(linearVelocity,msg_linearVelocity,mtxLinVel);
        safeSet(angularVelocity,msg_angularVelocity,mtxAngVel);
        safeSet(linearAcceleration,msg_linearAcceleration,mtxLinAcc);
        safeSet(angularAcceleration,msg_angularAcceleration,mtxAngAcc);
        logger.log(msg_time,{msg_position, msg_orientation, msg_linearVelocity,
            msg_angularVelocity, msg_linearAcceleration, msg_angularAcceleration});    
    }
}

Eigen::Vector3d Environment::getPosition()
{
    return safeGet(position,mtxPos);
}

Eigen::Vector3d Environment::getOrientation()
{
    return safeGet(orientation,mtxOri);
}

Eigen::Vector3d Environment::getLinearVelocity()
{
    return safeGet(linearVelocity,mtxLinVel);
}

Eigen::Vector3d Environment::getAngularVelocity()
{
    return safeGet(angularVelocity,mtxAngVel);
}

Eigen::Vector3d Environment::getLinearAcceleration()
{
    return safeGet(linearAcceleration,mtxLinAcc);
}

Eigen::Vector3d Environment::getAngularAcceleraton()
{
    return safeGet(angularAcceleration,mtxAngAcc);
}
