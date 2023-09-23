#pragma once

#include <Eigen/Dense>
#include <map>
#include "controller_mode.hpp"
#include "common.hpp"
#include "mixers.hpp"
#include "../communication/control.hpp"
#include "../navigation/NS.hpp"
#include "../communication/state.hpp"

class State;

class ControllerLoop
{
public:
    ControllerLoop(ControllerMode mode);
    virtual ~ControllerLoop() {}

    /// @brief Controller job that should be called in control loop.
    virtual void job(
        [[maybe_unused]] State* state,
        [[maybe_unused]] std::map<std::string,PID>& pids,
        [[maybe_unused]] Control& control,
        [[maybe_unused]] NS& navisys
        );

    /// @brief Convert joystick position to demands.
    virtual void handleJoystick(
        [[maybe_unused]] State* state,
        [[maybe_unused]] Eigen::Vector4d joystick
    ) {};

    /// @brief Prepare info about state and demands.
    virtual std::string demandInfo(
        [[maybe_unused]] State* state
        ) {return "ok";}

    /// @brief Return assigned mode.
    ControllerMode getMode() { return _mode; };



    static ControllerLoop* ControllerLoopFactory(ControllerMode mode);

protected:
    const ControllerMode _mode;
};