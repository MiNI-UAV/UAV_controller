#pragma once
#include <atomic>
#include <thread>
#include <zmq.hpp>
#include <functional>
#include "../controller/controller.hpp"

#define INFO_PERIOD 2

class Controller;

class State
{
    public:
        // std::atomic<double> demandedX = 0.0;
        // std::atomic<double> demandedY = 0.0;
        // std::atomic<double> demandedZ = 0.0;

	    // std::atomic<double> demandedFi = 0.0;
	    // std::atomic<double> demandedTheta = 0.0;
	    // std::atomic<double> demandedPsi = 0.0;

        // std::atomic<double> demandedU = 0.0;
        // std::atomic<double> demandedV = 0.0;
        // std::atomic<double> demandedW = 0.0;

        // std::atomic<double> demandedP = 0.0;
        // std::atomic<double> demandedQ = 0.0;
        // std::atomic<double> demandedR = 0.0;

        // std::atomic<double> throttle = 0.0;

        State(zmq::context_t* ctx, std::string uav_address, Controller* controller);
        ~State();
        std::string handleMsg(std::string msg);

    private:
        bool run;
        std::thread orderServer;
        Controller* _controller;

        void clearAll();
        std::string handleControl(std::string content);
        std::string handleMode(std::string content);
        std::string handleJoystick(std::string content);
};
