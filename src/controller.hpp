#pragma once
#include <map>
#include <string>
#include <Eigen/Dense>
#include <functional>
#include <optional>
#include "PID.hpp"
#include "GPS.hpp"
#include "gyro.hpp"
#include "mixers.hpp"
#include "control.hpp"
#include "PID.hpp"
#include "timed_loop.hpp"
#include "state.hpp"
#include "controller_mode.hpp"
#include "params.hpp"


class Controller
{
    public:
        Controller(zmq::context_t *ctx, std::string uav_address, Params& params);
        ~Controller();
        void run();
        void setMode(ControllerMode new_mode);
        void exitController();

    private:
        std::function<void()> jobs[4];
        ControllerMode mode;

        Status status;
        State state;
        GPS_AH gps;
	    Gyro gyro;
	    Control control;
        Params& params;
        std::optional<TimedLoop> loop;

        void syncWithPhysicEngine(zmq::context_t *ctx,std::string uav_address);
        void setCurrentDemands();
        void acroControllLoop();
        void angleControllLoop();
        void positionControllLoop();

};