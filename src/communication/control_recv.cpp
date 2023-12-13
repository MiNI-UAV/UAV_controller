#include "control.hpp"
#include <iostream>
#include "../defines.hpp"

std::string Control::handleMsg(std::string msg)
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

void Control::setMode(ControllerMode mode) 
{
    _controller->setMode(mode);
}

std::string Control::handleControl(std::string content)
{
    if(content.compare("exit") == 0)
    { 
        _controller->exitController();
        return "ok";
    }
    return "unknown";
}

std::string Control::handleMode(std::string content)
{
    try
    {
        auto mode = ControllerModeFromString(content.data());
        setMode(mode);
        return "ok";
    }
    catch(const std::exception& e)
    {
        return "unknown";
    }
}



std::string Control::handleJoystick(std::string content)
{
    static int i = 0;

    std::istringstream f(content);
    std::string value;
    std::vector<double> values;

    if(_controller->controller_loop == nullptr)
    {
        return "ok";
    }
    while (std::getline(f, value, ',')) {
        values.push_back(std::stod(value));
    }
    Eigen::Map<Eigen::VectorXd> joystick_values(values.data(), values.size());
    _controller->controller_loop->handleJoystick(joystick_values);
    if( i++ < def::INFO_PERIOD ) return "ok";
    i = 0;
    return _controller->controller_loop->demandInfo();
}