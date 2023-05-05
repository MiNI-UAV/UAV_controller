#include "state.hpp"
#include <zmq.hpp>
#include <iostream>

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

State::State(zmq::context_t *ctx, int port)
{
    orderServer = std::thread(orderServerJob,ctx, port, [this](std::string msg) {this->handleMsg(msg);});
}

State::~State()
{
    orderServer.join();
}

void State::handleMsg(std::string msg)
{
    std::istringstream f(msg);
    std::string command;
    while(1)
    {
        if(!getline(f, command, ',')) break;
        switch(command.at(0))
        {
            case 'Z':
                demandedZ = std::stod(command.substr(2));
            break;
            case 'F':
                demandedFi = std::stod(command.substr(2));
            break;
            case 'T':
                demandedTheta = std::stod(command.substr(2));
            break;
            case 'P':
                demandedPsi = std::stod(command.substr(2));
            break;
        }
    }
}
