#pragma once

#include <zmq.hpp>
#include <Eigen/Dense>
#include <atomic>
#include <thread>
#include <functional>
#include "../controller/controller.hpp"

class Controller;

class Control
{
    public:
        Control(zmq::context_t* ctx, std::string uav_address);
        ~Control();
        void prepare();
        void start();
        void stop();
        void recv();
        void sendSpeed(Eigen::VectorXd speeds);
        void sendSurface(Eigen::VectorXd angels);
        void startJet(int index);
        void sendHinge(char type, int index, int hinge_index, double value);
        std::string handleMsg(std::string msg);

    private:
        void sendVectorXd(std::string prefix, Eigen::VectorXd vec);
        void sendString(std::string msg);
        std::string handleControl(std::string content);
        std::string handleMode(std::string content);
        std::string handleJoystick(std::string content);

        std::atomic_bool run;
        std::thread orderServer;
        zmq::socket_t sock;
        Controller* _controller;
};