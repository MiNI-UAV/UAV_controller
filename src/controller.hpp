#pragma once
#include <map>
#include <string>
#include <Eigen/Dense>
#include <functional>
#include <optional>
#include "UAV_NS/NS.hpp"
#include "mixers.hpp"
#include "control.hpp"
#include "state.hpp"
#include "controller_mode.hpp"
#include "params.hpp"
#include "UAV_NS/environment.hpp"
#include "UAV_NS/NS.hpp"


class Controller
{
    public:
        Controller(zmq::context_t *ctx, std::string uav_address);
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
        NS navisys;
	    Control control;
        std::optional<TimedLoop> loop;

        void syncWithPhysicEngine(zmq::context_t *ctx,std::string uav_address);
        void setCurrentDemands();
        void acroControllLoop();
        void angleControllLoop();
        void positionControllLoop();

};