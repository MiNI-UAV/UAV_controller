#include "control.hpp"
#include <iostream>

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
        _controller->setMode(mode);
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
    const int INFO_PERIOD = 2;

    std::istringstream f(content);
    std::string value;
    Eigen::Vector<double,8> values;
    values.setZero();
    for(int i = 0; i < 8; i++)
    {
        if(!getline(f, value, ',')) break;
        values[i] = std::stod(value);
    }

    if(_controller->controller_loop == nullptr)
    {
        return "ok";
    }
    _controller->controller_loop->handleJoystick(values);
    i++;
    if( i < INFO_PERIOD ) return "ok";
    i = 0;
    return _controller->controller_loop->demandInfo();
}