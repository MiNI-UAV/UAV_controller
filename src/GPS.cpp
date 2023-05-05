#include "GPS.hpp"
#include <iostream>
#include <thread>
#include <functional>

void posListenerJob(zmq::context_t *ctx, std::string address,std::function<void(std::string)> handleMsg)
{
    std::cout << "Starting GPS&AH pos listener: " + address + "\n";
    zmq::socket_t sock = zmq::socket_t(*ctx, zmq::socket_type::sub);
    sock.set(zmq::sockopt::conflate,1);
    sock.connect(address);
    sock.set(zmq::sockopt::subscribe, "pos:");
    bool run = true;
    while(run)
    {
        zmq::message_t msg;
        const auto res = sock.recv(msg, zmq::recv_flags::none);
        if(!res)
        {
            std::cerr << "GPS&AH pos listener recv error" << std::endl;
            return;
        } 
        std::string msg_str =  std::string(static_cast<char*>(msg.data()), msg.size());
        handleMsg(msg_str);
    }
    sock.close();
    std::cout << "Ending GPS&AH pos listener: " << address << std::endl;
}

void vnListenerJob(zmq::context_t *ctx, std::string address,std::function<void(std::string)> handleMsg)
{
    std::cout << "Starting GPS&AH vel listener: " + address + "\n";
    zmq::socket_t sock = zmq::socket_t(*ctx, zmq::socket_type::sub);
    sock.set(zmq::sockopt::conflate,1);
    sock.connect(address);
    sock.set(zmq::sockopt::subscribe, "vn:");
    bool run = true;
    while(run)
    {
        zmq::message_t msg;
        const auto res = sock.recv(msg, zmq::recv_flags::none);
        if(!res)
        {
            std::cerr << "GPS&AH vel listener recv error" << std::endl;
            return;
        } 
        std::string msg_str =  std::string(static_cast<char*>(msg.data()), msg.size());
        handleMsg(msg_str);
    }
    sock.close();
    std::cout << "Ending GPS&AH vel listener: " << address << std::endl;
}

GPS_AH::GPS_AH(zmq::context_t *ctx, std::string uav_address)
{
    pos.setZero();
    orientation.setZero();
    vel.setZero();
    uav_address = uav_address + "/state";
    posListener = std::thread(posListenerJob,ctx, uav_address, [this](std::string msg) {this->handlePosMsg(msg);});
    vnListener = std::thread(vnListenerJob,ctx, uav_address, [this](std::string msg) {this->handleVelMsg(msg);});
}

GPS_AH::~GPS_AH()
{
    posListener.join();
    vnListener.join();
}

Eigen::Vector3d GPS_AH::getGPSPos()
{
    std::lock_guard<std::mutex> guard(mtxPos);
    return pos;
}

Eigen::Vector3d GPS_AH::getGPSVel()
{
    std::lock_guard<std::mutex> guard(mtxVel);
    return vel;
}

Eigen::Vector3d GPS_AH::getAH()
{
    std::lock_guard<std::mutex> guard(mtxOri);
    return orientation;
}

void GPS_AH::setPosAndOrient(Eigen::Vector3d newPos, Eigen::Vector3d newOrientation)
{
    newPos = GPSPosError(newPos);
    newOrientation = AHError(newOrientation);

    mtxPos.lock();
    pos = newPos;
    mtxPos.unlock();

    mtxOri.lock();
    orientation = newOrientation;
    mtxOri.unlock();
}

void GPS_AH::setVel(Eigen::Vector3d newVel)
{
    newVel = GPSVelError(newVel);
    std::lock_guard<std::mutex> guard(mtxVel);
    vel = newVel;
}

Eigen::Vector3d GPS_AH::GPSPosError(Eigen::Vector3d pos)
{
    //TODO: add some error
    return pos;
}

Eigen::Vector3d GPS_AH::GPSVelError(Eigen::Vector3d vel)
{
    //TODO: add some error
    return vel;
}

Eigen::Vector3d GPS_AH::AHError(Eigen::Vector3d orientation)
{
    //TODO: add some error
    return orientation;
}

void GPS_AH::handlePosMsg(std::string msg)
{
    std::istringstream f(msg.substr(4));
    std::string s;
    Eigen::Vector3d pos,ori;
    int i; 
    for (i = 0; i < 6; i++)
    {
        if(!getline(f, s, ','))
        {
            std::cerr << "Invalid pos msg" << std::endl;
            break;
        }

        if(i < 3)
        {
            pos(i) = std::stod(s);
        }
        else
        {
            ori(i-3) = std::stod(s);
        }
    }
    if(i != 6)
    {
        std::cerr << "Invalid pos msg" << std::endl;
        return;
    }
    setPosAndOrient(pos,ori);
}

void GPS_AH::handleVelMsg(std::string msg)
{
    std::istringstream f(msg.substr(3));
    std::string s;
    Eigen::Vector3d vn;
    int i; 
    for (i = 0; i < 6; i++)
    {
        if(!getline(f, s, ','))
        {
            std::cerr << "Invalid vel msg" << std::endl;
            break;
        }

        if(i < 3)
        {
            vn(i) = std::stod(s);
        }
    }
    if(i != 6)
    {
        std::cerr << "Invalid vel msg" << std::endl;
        return;
    }
    setVel(vn);
}
