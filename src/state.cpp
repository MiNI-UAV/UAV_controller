#include "state.hpp"
#include <zmq.hpp>
#include <iostream>
#include <functional>
#include <numbers>
#include <cmath>
#include "controller_mode.hpp"

void orderServerJob(zmq::context_t *ctx, std::string uav_address, std::function<std::string(std::string)> handleMsg, bool& run)
{
    uav_address = uav_address +  "/steer";
    std::cout << "Starting Order server: " + uav_address + "\n";
    zmq::socket_t sock = zmq::socket_t(*ctx, zmq::socket_type::rep);
    sock.set(zmq::sockopt::rcvtimeo,200);
    sock.bind(uav_address);
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
        auto rep = handleMsg(msg_str);
        zmq::message_t message(rep.data(), rep.size());
        sock.send(message,zmq::send_flags::none);
    }
    sock.close();
}

State::State(zmq::context_t* ctx, std::string uav_address, const ControllerMode& mode, std::function<void(ControllerMode)> setControlMode, std::function<void()> exitController):
_mode{mode},
_setControlMode{setControlMode},
_exitController{exitController}
{
    run = true;
    orderServer = std::thread(orderServerJob,ctx, uav_address, [this](std::string msg) {return this->handleMsg(msg);},std::ref(run));
}

State::~State()
{
    run = false;
    orderServer.join();
    std::cout << "Exiting Order Server!" << std::endl;
}

std::string State::handleMsg(std::string msg)
{
    std::string content = msg.substr(2); 
    switch(msg.at(0))
    {
        case 'c':
            return handleControl(content);
        case 'j':
            return handleJoystick(content);
        case 'm':
            return handleMode(content);
    }
    return "unknown";
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

std::string State::handleControl(std::string content)
{
    if(content.compare("exit") == 0)
    { 
        _exitController();
        return "ok";
    }
    return "unknown";
}

std::string State::handleMode(std::string content)
{
    clearAll();
    if(content.compare("none") == 0)
    {
        _setControlMode(ControllerMode::none);
        return "ok";
    }
    if(content.compare("acro") == 0)
    {
        _setControlMode(ControllerMode::acro);
        return "ok";
    }
    if(content.compare("pos") == 0)
    {
        _setControlMode(ControllerMode::position);
        return "ok";
    }
    if(content.compare("angle") == 0)
    {
        _setControlMode(ControllerMode::angle);
        return "ok";
    }
    return "unknown";
}

double clampAngle(double angle)
{
    angle = std::fmod(angle + std::numbers::pi,2*std::numbers::pi);
    if (angle < 0)
        angle += 2*std::numbers::pi;
    return angle - std::numbers::pi;
}

std::string State::handleJoystick(std::string content)
{
    constexpr double angleLimit = std::numbers::pi/5.0;

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
    return demandedInfo();

    case ControllerMode::angle:
        demandedZ -= values[1]/10.0;
	    demandedFi = values[2]*angleLimit;
	    demandedTheta = -values[3]*angleLimit;
        demandedPsi = clampAngle(demandedPsi + values[0]/100.0);
    return demandedInfo();

    case ControllerMode::position:
        demandedZ -= values[1]/10.0;
        demandedPsi = clampAngle(demandedPsi + values[0]/100.0);
        demandedX += ((values[3]*angleLimit)*std::cos(demandedPsi) - (values[2]*angleLimit)*std::sin(demandedPsi))/2.0;
        demandedY += ((values[3]*angleLimit)*std::sin(demandedPsi) + (values[2]*angleLimit)*std::cos(demandedPsi))/2.0;
    return demandedInfo();

    case ControllerMode::none:
    return ToString(_mode);
    
    default:
        std::cerr << "Not implemented yet!" << std::endl;
    return "unknown";
    }
}

std::string State::demandedInfo() 
{
    static int i = 0;
    i++;
    if(i != INFO_PERIOD) return "ok";
    i = 0;

    std::stringstream ss;
    std::string s;
    ss.precision(3);
    ss << std::fixed << ToString(_mode) << ",";

    switch (_mode)
    {
        case ControllerMode::acro:
            ss << demandedP << "," << demandedQ << "," << demandedR << "," << throttle;
        break;

        case ControllerMode::angle:
            ss << demandedFi << "," << demandedTheta << "," << demandedPsi << "," << demandedZ;
        break;

        case ControllerMode::position:
            ss << demandedX << "," << demandedY << "," << demandedZ << "," << demandedPsi;
        break;

        default:
        break;
    }
    return ss.str(); 
}
