#pragma once
#include <map>
#include <string>
#include <Eigen/Dense>
#include <functional>
#include "PID.hpp"
#include "GPS.hpp"
#include "gyro.hpp"
#include "mixers.hpp"
#include "control.hpp"
#include "PID.hpp"
#include "timed_loop.hpp"
#include "state.hpp"

enum ControllerMode
{
    position = 1,
    angle = 2,
    acro = 3
};

class Controller
{
    public:
        Controller(zmq::context_t *ctx, std::string uav_address,int controlPort);
        ~Controller();
        void run();
        void setMode(ControllerMode new_mode);

    private:
        std::map<std::string,PID> pids;
        std::function<void()> jobs[4];
        ControllerMode mode;
        Status status;
        State state;
        const int step_time = 3;

        GPS_AH gps;
	    Gyro gyro;
	    Control control;

        void loadPIDs(std::string configPath);
};