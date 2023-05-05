#include "control.hpp"
#include <iostream>

Control::Control(zmq::context_t *ctx, std::string uav_address)
{
    std::string address = uav_address + "/control";
    std::cout << "Starting control socket: " << address << std::endl;
    sock = zmq::socket_t(*ctx, zmq::socket_type::req);
    sock.connect(address);
}

void Control::prepare()
{
    zmq::message_t message("c:ping",6);
    sock.send(message,zmq::send_flags::none);
    zmq::message_t response;
    auto res = sock.recv(response);
    if(!res || response.to_string_view().compare("pong") != 0)
    {
        std::cerr << "Ping error" << std::endl;
		exit(1);
    }
    std::cout << "Ready!\n";
}

void Control::start()
{
    zmq::message_t first_msg("c:start",7);
    sock.send(first_msg,zmq::send_flags::none);
    std::cout << "Start!\n";
}

void Control::stop()
{
    zmq::message_t first_msg("c:stop",6);
    sock.send(first_msg,zmq::send_flags::none);
    zmq::message_t response;
    auto res = sock.recv(response);
    if(!res && response.str().compare("ok") != 0)
    {
        std::cerr << "Stop error" << std::endl;
		exit(1);
    }
}

void Control::sendSpeed(Eigen::VectorXd speeds)
{
    static Eigen::IOFormat commaFormat(4, Eigen::DontAlignCols," ",",");

    zmq::message_t response;
    auto res = sock.recv(response);
    if(!res && response.str().compare("ok") != 0)
    {
        std::cerr << "Send speed error" << std::endl;
		exit(1);
    }

    std::stringstream ss;
    std::string s;
    ss.precision(3);
    ss << "s:"<< speeds.format(commaFormat);
    s = ss.str();
    //std::cout << s << std::endl;
    zmq::message_t message(s.data(), s.size());
    sock.send(message,zmq::send_flags::none);
}

Control::~Control()
{
    sock.close();
}
