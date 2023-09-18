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
    //recv();

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
    sendVectorXd("s:",speeds);
}

void Control::sendSurface(Eigen::VectorXd angels) 
{
    sendVectorXd("e:",angels);
}

void Control::startJet(int index) 
{
    static const char* prefix = "j:";

    std::stringstream ss;
    ss << prefix << index;
    sendString(ss.str());
}

void Control::sendHinge(char type, int index, int hinge_index, double value) 
{
    static const char* prefix = "h:";

    std::stringstream ss;
    ss << prefix <<type << index << hinge_index << value;
    sendString(ss.str());
}

Control::~Control()
{
    sock.close();
}

void Control::sendVectorXd(std::string prefix, Eigen::VectorXd vec) 
{
    static Eigen::IOFormat commaFormat(4, Eigen::DontAlignCols," ",",");

    vec = vec.unaryExpr([](auto d) {return std::abs(d) < 1e-4 ? 0.0 : d;});
    std::stringstream ss;
    std::string s;
    ss.precision(5);
    ss << prefix << vec.format(commaFormat);
    s = ss.str();
    sendString(ss.str());
}

void Control::sendString(std::string msg) 
{
    //std::cout << "[" << msg << "]" << std::endl;
    recv();
    zmq::message_t message(msg.data(), msg.size());
    sock.send(message,zmq::send_flags::none);
}

void Control::recv()
{
    try
    {
        zmq::message_t response;
        auto res = sock.recv(response);
        if(!res && response.str().compare("ok") != 0)
        {
            std::cerr << "Recv error" << std::endl;
		    exit(1);
        }
    }
    catch(const zmq::error_t &ze)
    {
        std::cerr << "Recv error:" + std::to_string(ze.num()) << std::endl;
    }  
}
