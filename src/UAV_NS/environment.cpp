#include "environment.hpp"
#include <zmq.hpp>
#include <thread>
#include <Eigen/Dense>
#include <mutex>
#include <vector>
#include <memory>
#include <iostream>
#include <initializer_list>
#include "../utils.hpp"
#include "common.hpp"
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
    vel_world_sock(*ctx,zmq::socket_type::sub),
    accel_sock(*ctx,zmq::socket_type::sub),
    logger("env.csv", 
    "time,PosX,PosY,PosZ,Roll,Pitch,Yaw,"
    "VelX,VelY,VelZ,OmX,OmY,OmZ,"
    "VelBX,VelBY,VelBZ,OmBX,OmBY,OmBZ,"
    "AccX,AccY,AccZ,EpsX,EpsY,EpsZ")
{
    for(auto& sensor: UAVparams::getSingleton()->sensors)
    {   
        if(sensor.name.compare("accelerometer") == 0)
            sensorsVec3d.insert(std::make_pair(sensor.name,std::make_unique<Accelerometer>(*this,sensor.sd,sensor.bias,sensor.refreshTime)));
        if(sensor.name.compare("gyroscope") == 0)
            sensorsVec3d.insert(std::make_pair(sensor.name,std::make_unique<Gyroscope>(*this,sensor.sd,sensor.bias,sensor.refreshTime)));
        if(sensor.name.compare("magnetometer") == 0)
            sensorsVec3d.insert(std::make_pair(sensor.name,std::make_unique<Magnetometer>(*this,sensor.sd,sensor.bias,sensor.refreshTime)));
        if(sensor.name.compare("barometer") == 0)
            sensors.insert(std::make_pair(sensor.name,std::make_unique<Barometer>(*this,sensor.sd,sensor.bias,sensor.refreshTime)));
        if(sensor.name.compare("GPS") == 0)
            sensorsVec3d.insert(std::make_pair(sensor.name,std::make_unique<GPS>(*this,sensor.sd,sensor.bias,sensor.refreshTime)));
        if(sensor.name.compare("GPSVel") == 0)
            sensorsVec3d.insert(std::make_pair(sensor.name,std::make_unique<GPSVel>(*this,sensor.sd,sensor.bias,sensor.refreshTime)));
    }
    
    uav_address += "/state";
    connectConflateSocket(time_sock, uav_address, "t:");
    connectConflateSocket(pos_sock, uav_address, "pos:");
    connectConflateSocket(vel_sock, uav_address, "vb:");
    connectConflateSocket(vel_world_sock, uav_address, "vn:");
    connectConflateSocket(accel_sock, uav_address, "ab:");
    run.store(true,std::memory_order_relaxed);
    listener = std::thread(&Environment::listenerJob, this);

    std::cout << "Initializing environment done" << std::endl;
}

Environment::~Environment()
{
    run.store(false,std::memory_order_relaxed);
    listener.join();

    time_sock.close();
    pos_sock.close();
    vel_sock.close();
    vel_world_sock.close();
    accel_sock.close();
    std::cout << "Env exited." << std::endl;
}

double Environment::getTime()
{
    return time.load();
}

template <int Size1, int Size2>
bool recvVectors(zmq::socket_t& sock, int skip, Eigen::Vector<double,Size1>& vec1, Eigen::Vector<double,Size2>& vec2)
{
    zmq::message_t msg;
    if(!sock.recv(msg, zmq::recv_flags::none))
    {
        if(zmq_errno() != EAGAIN) std::cerr << " listener recv error" << std::endl;
        return true;
    }
    std::string msg_str =  std::string(static_cast<char*>(msg.data()), msg.size());
    //std::cout << "[" << msg_str << "]" << std::endl;
    std::istringstream f(msg_str.substr(skip));
    std::string s;
    int i; 
    for (i = 0; i < Size1 + Size2; i++)
    {
        if(!getline(f, s, ','))
        {
            std::cerr << "Invalid msg" << std::endl;
            break;
        }
        if(i < Size1)
        {
            vec1(i) = std::stod(s);
        }
        else
        {
            vec2(i-Size1) = std::stod(s);
        }
    }
    if(i != Size1 + Size2)
    {
        std::cerr << "Invalid msg" << std::endl;
        return true;
    }
    return false;
}

Eigen::Matrix<double, 3, 3> r_nb(const Eigen::Vector3d&  RPY)
{
    double fi = RPY(0);
    double theta = RPY(1);
    double psi = RPY(2);
    Eigen::Matrix<double, 3, 3> r_nb;
    r_nb << cos(theta)*cos(psi),                            cos(theta)*sin(psi),                           -sin(theta),
            sin(fi)*sin(theta)*cos(psi) - cos(fi)*sin(psi), sin(fi)*sin(theta)*sin(psi) + cos(fi)*cos(psi), sin(fi)*cos(theta),
            cos(fi)*sin(theta)*cos(psi) + sin(fi)*sin(psi), cos(fi)*sin(theta)*sin(psi) - sin(fi)*cos(psi), cos(fi)*cos(theta);
    return r_nb;
}

Eigen::Matrix<double, 3, 3> r_nb(const Eigen::Vector4d&  e)
{
    Eigen::Matrix<double, 3, 3> r_nb;
    r_nb << e(0)*e(0)+e(1)*e(1)-e(2)*e(2)-e(3)*e(3)  , 2*(e(1)*e(2)+e(0)*e(3))                   , 2*(e(1)*e(3)-e(0)*e(2)),
            2*(e(1)*e(2)-e(0)*e(3))                  , e(0)*e(0)-e(1)*e(1)+e(2)*e(2)-e(3)*e(3)   , 2*(e(2)*e(3)+e(0)*e(1)),
            2*(e(1)*e(3)+e(0)*e(2))                  , 2*(e(2)*e(3)-e(0)*e(1))                   , e(0)*e(0)-e(1)*e(1)-e(2)*e(2)+e(3)*e(3);
    return r_nb;
}


void Environment::listenerJob() 
{
    double msg_time;
    Eigen::Vector3d msg_position;
#ifdef USE_QUATERIONS
    Eigen::Vector4d msg_orientation;
#else
    Eigen::Vector3d msg_orientation;
#endif
    Eigen::Vector3d msg_worldLinearVelocity;
    Eigen::Vector3d msg_worldAngularVelocity;
    Eigen::Vector3d msg_linearVelocity;
    Eigen::Vector3d msg_angularVelocity;
    Eigen::Vector3d msg_linearAcceleration;
    Eigen::Vector3d msg_angularAcceleration;
    Eigen::Matrix3d msg_r_nb;

    while(run.load())
    {
        zmq::message_t msg;
        if(!time_sock.recv(msg, zmq::recv_flags::none))
        {
            if(zmq_errno() != EAGAIN) std::cerr << " listener recv error" << std::endl;
            continue;
        }
        std::string msg_str =  std::string(static_cast<char*>(msg.data()), msg.size());
        //std::cout << "[" << msg_str << "]" << std::endl;
        msg_time = std::stod(msg_str.substr(2));
        if(recvVectors(pos_sock,4,msg_position,msg_orientation)) continue;
        if(recvVectors(vel_sock,3,msg_linearVelocity,msg_angularVelocity)) continue;
        if(recvVectors(vel_world_sock,3,msg_worldLinearVelocity,msg_worldAngularVelocity)) continue;
        if(recvVectors(accel_sock,3,msg_linearAcceleration,msg_angularAcceleration)) continue;
        msg_r_nb = r_nb(msg_orientation);

        time.store(msg_time,std::memory_order_consume);
        safeSet(position,msg_position,mtxPos);
        safeSet(orientation,msg_orientation,mtxOri);
        safeSet(R_nb,msg_r_nb,mtxRnb);
        safeSet(worldLinearVelocity,msg_worldLinearVelocity,mtxWorldLinVel);
        safeSet(worldAngularVelocity,msg_worldAngularVelocity,mtxWorldAngVel);
        safeSet(linearVelocity,msg_linearVelocity,mtxLinVel);
        safeSet(angularVelocity,msg_angularVelocity,mtxAngVel);
        safeSet(linearAcceleration,msg_linearAcceleration,mtxLinAcc);
        safeSet(angularAcceleration,msg_angularAcceleration,mtxAngAcc);
        logger.log(msg_time,{msg_position, msg_orientation,
                   msg_worldLinearVelocity, msg_worldAngularVelocity,
                   msg_linearVelocity, msg_angularVelocity,
                   msg_linearAcceleration, msg_angularAcceleration});    
    }
}

Eigen::Vector3d Environment::getPosition()
{
    return safeGet(position,mtxPos);
}

#ifdef USE_QUATERIONS
Eigen::Vector4d Environment::getOrientation()
#else
Eigen::Vector3d Environment::getOrientation()
#endif
{
    return safeGet(orientation,mtxOri);
}

Eigen::Vector3d Environment::getWorldLinearVelocity() 
{
  return safeGet(worldLinearVelocity,mtxWorldLinVel);
}

Eigen::Vector3d Environment::getWorldAngularVelocity()
{
  return safeGet(worldAngularVelocity,mtxWorldAngVel);
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
    for (auto& [_, value]: sensors)
    {
        value->update();
    }
    for (auto& [_, value]: sensorsVec3d)
    {
        value->update();
    }
}
