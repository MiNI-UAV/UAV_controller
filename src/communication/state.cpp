#include "state.hpp"
#include <zmq.hpp>
#include <Eigen/Dense>
#include <iostream>
#include <functional>
#include <numbers>
#include <cmath>
#include "../controller/controller_mode.hpp"
#include "../controller/controller_loop.hpp"

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

State::State(zmq::context_t *ctx, std::string uav_address,
             Controller* controller)
    : _controller{controller} {
  run = true;
  orderServer = std::thread(
      orderServerJob, ctx, uav_address,
      [this](std::string msg) { return this->handleMsg(msg); }, std::ref(run));
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
        _controller->exitController();
        return "ok";
    }
    return "unknown";
}

std::string State::handleMode(std::string content)
{
    clearAll();
    try
    {
        auto mode = ControllerModeFromString(content.data());
        _controller->setMode(mode);
        return "ok";
    }
    catch(const std::exception& e)
    {
        return "unknown";
    }
}



std::string State::handleJoystick(std::string content)
{
    static int i = 0;

    std::istringstream f(content);
    std::string value;
    Eigen::Vector4d values;
    for(int i = 0; i < 4; i++)
    {
        if(!getline(f, value, ',')) break;
        values[i] = (std::stoi(value)-512)/512.0;
    }

    if(_controller->controller_loop == nullptr)
    {
        return "ok";
    }
    _controller->controller_loop->handleJoystick(this,values);
    i++;
    if( i < INFO_PERIOD ) return "ok";
    i = 0;
    return _controller->controller_loop->demandInfo(this);
}
