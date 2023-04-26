#include "GPS.hpp"
#include <iostream>
#include <thread>
#include <functional>

void listenerJob(zmq::context_t *ctx, std::string address,std::function<void(std::string)> handleMsg)
{
    std::cout << "Starting GPS&AH listener: " << address << std::endl;
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
            std::cerr << "GPS&AH listener recv error" << std::endl;
            return;
        } 
        std::string msg_str =  std::string(static_cast<char*>(msg.data()), msg.size());
        handleMsg(msg_str);
    }
    sock.close();
    std::cout << "Ending GPS&AH listener: " << address << std::endl;
}

GPS_AH::GPS_AH(zmq::context_t *ctx, std::string uav_address)
{
    pos.setZero();
    orientation.setZero();
    listener = std::thread(listenerJob,ctx, uav_address, [this](std::string msg) {this->handleMsg(msg);});
}

GPS_AH::~GPS_AH()
{
    listener.join();
}

Eigen::Vector3d GPS_AH::getGPS()
{
    std::lock_guard<std::mutex> guard(mtx);
    return pos;
}

Eigen::Vector3d GPS_AH::getAH()
{
    std::lock_guard<std::mutex> guard(mtx);
    return orientation;
}

void GPS_AH::setPosAndOrient(Eigen::Vector3d newPos, Eigen::Vector3d newOrientation)
{
    newPos = GPSError(newPos);
    newOrientation = AHError(newOrientation);
    std::lock_guard<std::mutex> guard(mtx);
    pos = newPos;
    orientation = newOrientation;
}

Eigen::Vector3d GPS_AH::GPSError(Eigen::Vector3d pos)
{
    //TODO: add some error
    return pos;
}

Eigen::Vector3d GPS_AH::AHError(Eigen::Vector3d orientation)
{
    //TODO: add some error
    return orientation;
}

void GPS_AH::handleMsg(std::string msg)
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
