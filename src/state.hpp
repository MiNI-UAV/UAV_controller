#pragma once
#include <atomic>
#include <thread>
#include <zmq.hpp>
#include <functional>
#include "UAV_common/status.hpp"
#include "controller_mode.hpp"

class State
{
    public:
        std::atomic<double> demandedX = 0.0;
        std::atomic<double> demandedY = 0.0;
        std::atomic<double> demandedZ = 0.0;

	    std::atomic<double> demandedFi = 0.0;
	    std::atomic<double> demandedTheta = 0.0;
	    std::atomic<double> demandedPsi = 0.0;

        std::atomic<double> demandedU = 0.0;
        std::atomic<double> demandedV = 0.0;
        std::atomic<double> demandedW = 0.0;

        std::atomic<double> demandedP = 0.0;
        std::atomic<double> demandedQ = 0.0;
        std::atomic<double> demandedR = 0.0;

        std::atomic<double> throttle = 0.0;

        State(zmq::context_t* ctx, std::string uav_address, const ControllerMode& mode, std::function<void(ControllerMode)> setControlMode, std::function<void()> exitController);
        ~State();
        void handleMsg(std::string msg);

    private:
        bool run;
        std::thread orderServer;
        const ControllerMode& _mode;
        std::function<void(ControllerMode)> _setControlMode;
        std::function<void()> _exitController;


        void clearAll();
        void handleControl(std::string content);
        void handleMode(std::string content);
        void handleJoystick(std::string content);
};
