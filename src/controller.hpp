#pragma once
#include <map>
#include <string>
#include <Eigen/Dense>
#include <functional>
#include <optional>
#include "UAV_common/PID.hpp"
#include "UAV_NS/NS.hpp"
#include "mixers.hpp"
#include "control.hpp"
#include "UAV_common/timed_loop.hpp"
#include "state.hpp"
#include "controller_mode.hpp"
#include "params.hpp"
#include "UAV_NS/environment.hpp"
#include "UAV_NS/NS.hpp"


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
        Environment env;
        //NS navisys;
        NS navisys2;
	    Control control;
        Params& params;
        std::optional<TimedLoop> loop;

        void syncWithPhysicEngine(zmq::context_t *ctx,std::string uav_address);
        void setCurrentDemands();
        void acroControllLoop();
        void angleControllLoop();
        void positionControllLoop();

};