#include "state.hpp"
#include <zmq.hpp>
#include <iostream>
#include <functional>
#include <numbers>
#include <cmath>
#include "controller_mode.hpp"
#include "status.hpp"

void orderServerJob(zmq::context_t *ctx, std::string uav_address, std::function<void(std::string)> handleMsg, bool& run)
{
    uav_address = uav_address +  "/steer";
    std::cout << "Starting Order server: " + uav_address + "\n";
    zmq::socket_t sock = zmq::socket_t(*ctx, zmq::socket_type::sub);
    sock.set(zmq::sockopt::rcvtimeo,200);
    sock.bind(uav_address);
    //TODO: Remove below temporiary solution
    sock.bind("tcp://*:"+std::to_string(1234));
    sock.set(zmq::sockopt::subscribe, "");
    run = true;
    while(run)
    {
        zmq::message_t msg;
        const auto res = sock.recv(msg, zmq::recv_flags::none);
        if(!res)
        {
            if(zmq_errno() != EAGAIN) std::cerr << "Order server recv error" << std::endl;
            continue;
        } 
        std::string msg_str =  std::string(static_cast<char*>(msg.data()), msg.size());
        handleMsg(msg_str);
    }
    sock.close();
}

State::State(zmq::context_t* ctx, std::string uav_address, const ControllerMode& mode, std::function<void(ControllerMode)> setControlMode, std::function<void()> exitController):
_mode{mode},
_setControlMode{setControlMode},
_exitController{exitController}
{
    run = true;
    orderServer = std::thread(orderServerJob,ctx, uav_address, [this](std::string msg) {this->handleMsg(msg);},std::ref(run));
}

State::~State()
{
    run = false;
    orderServer.join();
    std::cout << "Exiting Order Server!" << std::endl;
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

double clampAngle(double angle)
{
    angle = std::fmod(angle + std::numbers::pi,2*std::numbers::pi);
    if (angle < 0)
        angle += 2*std::numbers::pi;
    return angle - std::numbers::pi;
}

void State::handleJoystick(std::string content)
{
    constexpr double angleLimit = std::numbers::pi/6.0;
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
        throttle = values[1];
        demandedP = values[2]*5.0;
	    demandedQ = -values[3]*5.0;
	    demandedR = values[0]*3.0;

    break;

    case ControllerMode::angle:
        demandedZ -= values[1]/10.0;
	    demandedFi = values[2]*angleLimit;
	    demandedTheta = -values[3]*angleLimit;
        demandedPsi = clampAngle(demandedPsi + values[0]/100.0);
    break;

    case ControllerMode::position:
        demandedZ -= values[1]/10.0;
        demandedPsi = clampAngle(demandedPsi + values[0]/100.0);
        demandedX += ((values[3]*angleLimit)*std::cos(demandedPsi) - (values[2]*angleLimit)*std::sin(demandedPsi))/2.0;
        demandedY += ((values[3]*angleLimit)*std::sin(demandedPsi) + (values[2]*angleLimit)*std::cos(demandedPsi))/2.0;
    break;

    case ControllerMode::none:
    break;
    
    default:
        std::cerr << "Not implemented yet!" << std::endl;
    }
}
