#include "control.hpp"
#include <iostream>

Control::Control(zmq::context_t *ctx, std::string uav_address)
{
    std::string address = uav_address + "/control";
    std::cout << "Starting GPS&AH pos listener: " << address << std::endl;
    sock = zmq::socket_t(*ctx, zmq::socket_type::pub);
    sock.connect(address);
}

Control::~Control()
{
    sock.close();
}
