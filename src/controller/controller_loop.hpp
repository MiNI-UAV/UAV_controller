#pragma once

#include <Eigen/Dense>
#include <map>
#include "controller_mode.hpp"
#include "common.hpp"
#include "mixers.hpp"
#include "../communication/control.hpp"
#include "../navigation/NS.hpp"
#include "../communication/state.hpp"

class ControllerLoop
{
public:
    ControllerLoop(ControllerMode mode);
    virtual ~ControllerLoop() {}

    /// @brief Controller job that should be called in control loop.
    virtual void job(
        [[maybe_unused]] std::map<std::string,PID>& pids,
        [[maybe_unused]] Control& control,
        [[maybe_unused]] NS& navisys
        );

    /// @brief Convert joystick position to demands.
    virtual void handleJoystick(
        [[maybe_unused]] Eigen::Vector4d joystick
    ) 
    {};

    /// @brief Prepare info about state and demands.
    virtual std::string demandInfo() 
    {
        return "ERROR";
    }

    /// @brief Define pids controller required to work
    virtual const std::vector<std::string>& requiredPIDs()
    {
        return required_pids;
    };

    /// @brief Define pids controller required to work
    virtual void overridePosition(
        [[maybe_unused]] Eigen::Vector3d position,
        [[maybe_unused]] Eigen::Vector3d orientation
    )
    {};

    /// @brief Return assigned mode.
    ControllerMode getMode() { return _mode; };

    static ControllerLoop* ControllerLoopFactory(ControllerMode mode);

protected:
    const ControllerMode _mode;
    std::vector<std::string> required_pids;
};