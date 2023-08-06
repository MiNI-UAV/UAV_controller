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
#include "sensors.hpp"

void connectConflateSocket(zmq::socket_t& sock, std::string address, std::string topic)
{
    sock.set(zmq::sockopt::rcvtimeo,1);
    sock.set(zmq::sockopt::conflate,1);
    sock.set(zmq::sockopt::subscribe, topic);
    sock.connect(address);
}

Environment::Environment(zmq::context_t *ctx, std::string uav_address):
    time_sock(*ctx,zmq::socket_type::sub),
    pos_sock(*ctx,zmq::socket_type::sub),
    vel_sock(*ctx,zmq::socket_type::sub),
    accel_sock(*ctx,zmq::socket_type::sub),
    logger("env.csv", 
    "time,PosX,PosY,PosZ,Roll,Pitch,Yaw,"
    "VelX,VelY,VelZ,OmX,OmY,OmZ,"
    "AccX,AccY,AccZ,EpsX,EpsY,EpsZ"),

    acc(*this,0.0),
    gyro(*this,0.0),
    mag(*this, 0.0),
    baro(*this, 0.0),
    gps(*this,0.0),
    gpsVel(*this, 0.0)
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
    std::cout << "Env exited." << std::endl;
}

double Environment::getTime()
{
    return time.load();
}

bool recvVectors(zmq::socket_t& sock, int skip, Eigen::Vector3d& vec1, Eigen::Vector3d& vec2)
{
    zmq::message_t msg;
    if(!sock.recv(msg, zmq::recv_flags::none))
    {
        if(zmq_errno() != EAGAIN) std::cerr << " listener recv error" << std::endl;
        return true;
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
        return true;
    }
    return false;
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
            if(zmq_errno() != EAGAIN) std::cerr << " listener recv error" << std::endl;
            continue;
        }
        std::string msg_str =  std::string(static_cast<char*>(msg.data()), msg.size());
        msg_time = std::stod(msg_str.substr(2));
        if(recvVectors(pos_sock,4,msg_position,msg_orientation)) continue;
        if(recvVectors(vel_sock,3,msg_linearVelocity,msg_angularVelocity)) continue;
        if(recvVectors(accel_sock,3,msg_linearAcceleration,msg_angularAcceleration)) continue;

        time.store(msg_time,std::memory_order_consume);
        safeSet(position,msg_position,mtxPos);
        safeSet(orientation,msg_orientation,mtxOri);

        double cf = cos(msg_orientation(0));
        double sf = sin(msg_orientation(0));
        double ct = cos(msg_orientation(1));
        double st = sin(msg_orientation(1));
        double cp = cos(msg_orientation(2));
        double sp = sin(msg_orientation(2));
        mtxRnb.lock();
        R_nb << ct*cp           , ct*sp           , -st,
                sf*st*cp - cf*sp, sf*st*sp + cf*cp, sf*ct,
                cf*st*cp + sf*sp, cf*st*sp - sf*cp, cf*ct;
        mtxRnb.unlock();

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

Eigen::Matrix3d Environment::getRnb()
{
    return safeGet(R_nb, mtxRnb);
}

void Environment::updateSensors() 
{
    acc.update();
    gyro.update();
    mag.update();
    baro.update();
}
