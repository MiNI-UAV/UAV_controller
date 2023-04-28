#include "control.hpp"
#include <iostream>

Control::Control(zmq::context_t *ctx, std::string uav_address)
{
    std::string address = uav_address + "/control";
    std::cout << "Starting control socket: " << address << std::endl;
    sock = zmq::socket_t(*ctx, zmq::socket_type::pub);
    sock.connect(address);
}

void Control::sendSpeed(Eigen::VectorXd speeds)
{
    static Eigen::IOFormat commaFormat(4, Eigen::DontAlignCols," ",",");
    std::stringstream ss;
    std::string s;
    ss << "s:"<< speeds.format(commaFormat);
    s = ss.str();
    zmq::message_t message(s.data(), s.size());
    sock.send(message,zmq::send_flags::none);
}

Control::~Control()
{
    sock.close();
}
