#include "control.hpp"
#include <iostream>

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

Control::Control(zmq::context_t *ctx, std::string uav_address)
{
    std::string address = uav_address + "/control";
    std::cout << "Starting control socket: " << address << std::endl;
    sock = zmq::socket_t(*ctx, zmq::socket_type::req);
    sock.connect(address);
    run = true;
    // orderServer = std::thread(
    //     orderServerJob, ctx, uav_address,
    //     std::bind_front(&Control::handleMsg, this), std::ref(run));
}

Control::~Control()
{
    run = false;
    orderServer.join();
    std::cout << "Exiting Order Server!" << std::endl;
    sock.close();
}