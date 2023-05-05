#include "state.hpp"
#include <zmq.hpp>
#include <iostream>
#include <functional>
#include <numbers>
#include "controller_mode.hpp"
#include "status.hpp"

void orderServerJob(zmq::context_t *ctx, int port, std::function<void(std::string)> handleMsg)
{
    std::cout << "Starting Order server on port: " + std::to_string(port) + "\n";
    zmq::socket_t sock = zmq::socket_t(*ctx, zmq::socket_type::sub);
    sock.bind("tcp://*:"+std::to_string(port));
    sock.set(zmq::sockopt::subscribe, "");
    bool run = true;
    while(run)
    {
        zmq::message_t msg;
        const auto res = sock.recv(msg, zmq::recv_flags::none);
        if(!res)
        {
            std::cerr << "Order server recv error" << std::endl;
            return;
        } 
        std::string msg_str =  std::string(static_cast<char*>(msg.data()), msg.size());
        handleMsg(msg_str);
    }
    sock.close();
    std::cout << "Ending Order server" << std::endl;
}

State::State(zmq::context_t* ctx, int port, const ControllerMode& mode, std::function<void(ControllerMode)> setControlMode, std::function<void()> exitController):
_mode{mode},
_setControlMode{setControlMode},
_exitController{exitController}
{
    orderServer = std::thread(orderServerJob,ctx, port, [this](std::string msg) {this->handleMsg(msg);});
}

State::~State()
{
    orderServer.join();
}

void State::handleMsg(std::string msg)
{
    std::string content = msg.substr(2); 
    switch(msg.at(0))
    {
        case 'c':
            handleControl(content);
        break;
        case 'j':
            handleJoystick(content);
        break;
        case 'm':
            handleMode(content);
        break;

    }
}

void State::clearAll()
{
    demandedX = 0.0;
    demandedY = 0.0;
    demandedZ = 0.0;

    demandedFi = 0.0;
    demandedTheta = 0.0;
    demandedPsi = 0.0;

    demandedU = 0.0;
    demandedV = 0.0;
    demandedW = 0.0;

    demandedP = 0.0;
    demandedQ = 0.0;
    demandedR = 0.0;

    throttle = 0.0;
}

void State::handleControl(std::string content)
{
    if(content.compare("exit") == 0) _exitController();
}

void State::handleMode(std::string content)
{
    clearAll();
    if(content.compare("none") == 0)
    {
        _setControlMode(ControllerMode::none);
        return;
    }
    if(content.compare("acro") == 0)
    {
        _setControlMode(ControllerMode::acro);
        return;
    }
    if(content.compare("pos") == 0)
    {
        _setControlMode(ControllerMode::position);
        return;
    }
    if(content.compare("angle") == 0)
    {
        _setControlMode(ControllerMode::angle);
        return;
    }
}

void State::handleJoystick(std::string content)
{
    std::istringstream f(content);
    std::string value;
    double values[4];
    for(int i = 0; i < 4; i++)
    {
        if(!getline(f, value, ',')) break;
        values[i] = (std::stoi(value)-512)/512.0;
    }
    switch (_mode)
    {
    case ControllerMode::acro:
        throttle = (values[1]+1.0)/2.0;
        demandedP = values[2];
	    demandedQ = values[3];
	    demandedR = values[0];

    break;

    case ControllerMode::angle:
        demandedZ += values[1]/1000.0;
	    demandedFi = values[2];
	    demandedTheta = values[3];
	    demandedPsi += values[0]/1000.0;
    break;
    
    default:
        std::cerr << "Not implemented yet!" << std::endl;
    }
}
