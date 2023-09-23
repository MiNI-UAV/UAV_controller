#pragma once
#include <map>
#include <string>
#include <Eigen/Dense>
#include <functional>
#include <optional>
#include "UAV_NS/NS.hpp"
#include "mixers.hpp"
#include "control.hpp"
#include "Controller/controller_mode.hpp"
#include "Controller/controller_loop.hpp"
#include "common.hpp"
#include "UAV_NS/environment.hpp"
#include "state.hpp"

class State;
class ControllerLoop;

class Controller
{
    friend class State;

    public:
        Controller(zmq::context_t *ctx, std::string uav_address, ControllerMode initialMode);
        ~Controller();
        void run();
        void setMode(ControllerMode new_mode);
        void exitController();

    private:
        ControllerLoop* controller_loop;
        State* state;
        Status status;
        Environment env;
        NS navisys;
	    Control control;
        std::optional<TimedLoop> loop;
        std::map<std::string,PID> pids;

        void syncWithPhysicEngine(zmq::context_t *ctx,std::string uav_address);
        void setCurrentDemands();
        void acroControllLoop();
        void angleControllLoop();
        void positionControllLoop();

};