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

/// @brief Central controller class
class ControlSystem
{
    friend class Control;

    public:
        /// @brief Constructor
        /// @param ctx zero mq context
        /// @param uav_address address of simulation sockets
        ControlSystem(zmq::context_t *ctx, std::string uav_address);

        // @brief Deconstructor
        ~ControlSystem();

        /// @brief Run controller
        void run();

        /// @brief Change controller mode
        /// @param new_mode new contoller mode
        void setMode(ControllerMode new_mode);

        /// @brief Stop controller loop
        void exitController();

    private:
        ControllerLoop* controller_loop;
        Control* control;
        Status status;
        Environment env;
        NS navisys;
        std::optional<TimedLoop> loop;
        std::map<std::string,PID> pids;

        /// @brief Starts controller loop
        void startLoop();


        /// @brief Synchronize start with physic engine
        /// @param ctx zero mq context
        /// @param uav_address address of simulation sockets
        void syncWithPhysicEngine(zmq::context_t *ctx,std::string uav_address);
};