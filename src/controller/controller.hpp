#pragma once
#include <map>
#include <string>
#include <Eigen/Dense>
#include <functional>
#include <optional>
#include "../navigation/NS.hpp"
#include "../navigation/environment.hpp"
#include "mixers.hpp"
#include "controller_mode.hpp"
#include "controller_loop.hpp"
#include "common.hpp"
#include "../communication/control.hpp"


class ControllerLoop;
class Control;

class Controller
{
    friend class Control;

    public:
        Controller(zmq::context_t *ctx, std::string uav_address);
        ~Controller();
        void run();
        void setMode(ControllerMode new_mode);
        void exitController();

    private:
        ControllerLoop* controller_loop;
        Control* control;
        Status status;
        Environment env;
        NS navisys;
        std::optional<TimedLoop> loop;
        std::map<std::string,PID> pids;

        void startLoop();
        void syncWithPhysicEngine(zmq::context_t *ctx,std::string uav_address);
};