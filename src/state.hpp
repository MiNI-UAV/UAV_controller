#pragma once
#include <atomic>
#include <thread>
#include <zmq.hpp>

class State
{
    public:
        std::atomic<double> demandedZ = 0.0;
	    std::atomic<double> demandedFi = 0.0;
	    std::atomic<double> demandedTheta = 0.0;
	    std::atomic<double> demandedPsi = 0.0;

        State(zmq::context_t* ctx, int port);
        ~State();
        void handleMsg(std::string msg);

    private:
        std::thread orderServer;

};