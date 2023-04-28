#include "gyro.hpp"
#include <iostream>
#include <Eigen/Dense>


void angularVelListenerJob(zmq::context_t *ctx, std::string address,std::function<void(std::string)> handleMsg)
{
    std::cout << "Starting Gyro listener: " << address << std::endl;
    zmq::socket_t sock = zmq::socket_t(*ctx, zmq::socket_type::sub);
    sock.set(zmq::sockopt::conflate,1);
    sock.connect(address);
    sock.set(zmq::sockopt::subscribe, "vb:");
    bool run = true;
    while(run)
    {
        zmq::message_t msg;
        const auto res = sock.recv(msg, zmq::recv_flags::none);
        if(!res)
        {
            std::cerr << "Gyro listener recv error" << std::endl;
            return;
        } 
        std::string msg_str =  std::string(static_cast<char*>(msg.data()), msg.size());
        std::cout << msg_str << "\n";
        handleMsg(msg_str);
    }
    sock.close();
    std::cout << "Ending Gyro listener: " << address << std::endl;
}

Gyro::Gyro(zmq::context_t *ctx, std::string uav_address)
{
    angularVel.setZero();
    angularVelListener = std::thread(angularVelListenerJob,ctx, uav_address, [this](std::string msg) {this->handleVelMsg(msg);});
}

Gyro::~Gyro()
{
    angularVelListener.join();
}

Eigen::Vector3d Gyro::getAngularVel()
{
    return angularVel;
}

void Gyro::handleVelMsg(std::string msg)
{
    std::istringstream f(msg.substr(3));
    std::string s;
    Eigen::Vector3d angularVel;
    int i; 
    for (i = 0; i < 6; i++)
    {
        if(!getline(f, s, ','))
        {
            std::cerr << "Invalid vel msg" << std::endl;
            break;
        }

        if(i >= 3)
        {
            angularVel(i-3) = std::stod(s);
        }
    }
    if(i != 6)
    {
        std::cerr << "Invalid vel msg" << std::endl;
        return;
    }
    setAngularVel(angularVel);
}

void Gyro::setAngularVel(Eigen::Vector3d newVel)
{
    newVel = gyroError(newVel);
    std::lock_guard<std::mutex> guard(mtxAngVel);
    angularVel = newVel;
}

Eigen::Vector3d Gyro::gyroError(Eigen::Vector3d angularVel)
{
    //TODO: add some error
    return angularVel;
}
