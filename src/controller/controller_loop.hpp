#pragma once

#include <Eigen/Dense>
#include <map>
#include "controller_mode.hpp"
#include "common.hpp"
#include "mixers.hpp"
#include "../communication/control.hpp"
#include "../navigation/NS.hpp"

class Control;

/// @brief This class is interface of controller modes. All modes should keep this strucure and implements all true virtual methods.
class ControllerLoop
{
public:
    /// @brief Base class constructor
    /// @param mode mode enum value
    ControllerLoop(ControllerMode mode);
    
    /// @brief Virtual deconstructor for defined behavior
    virtual ~ControllerLoop() {}


    /// @brief Controller job that will be called in control loop.
    /// @param controllers map of aviliable controllers
    /// @param control reference to control instatce that is used to send control commands
    /// @param navisys navigation system reference
    virtual void job(
        [[maybe_unused]] std::map<std::string,std::unique_ptr<Controller>>& controllers,
        [[maybe_unused]] Control& control,
        [[maybe_unused]] NS& navisys
        );

    /// @brief Handle incomming joystick deflaction
    /// @param joystick joystick axes deflaction
    virtual void handleJoystick(
        [[maybe_unused]] Eigen::VectorXd joystick
    ) 
    {};

    /// @brief Prepare info about state and demands.
    /// @return information about mode and actually set demands
    virtual std::string demandInfo() 
    {
        return ControllerModeToString(_mode);
    }

    /// @brief Defines controllers controller required by mode
    /// @return vector of names of required controllers
    virtual const std::vector<std::string>& requiredcontrollers()
    {
        return required_controllers;
    };

    /// @brief Overrides demands to apply to given postion and orientation
    /// @param position position vector in world frame
    /// @param orientation orientation vector in world frame
    virtual void overridePosition(
        [[maybe_unused]] Eigen::Vector3d position,
        [[maybe_unused]] Eigen::Vector3d orientation
    )
    {};

    /// @brief Returns assigned mode enum value.
    /// @return mode enum value
    ControllerMode getMode() { return _mode; };

    /// @brief ControllerLoop factor. Returns instace of ControllerLoop that implements specified mode
    /// @param mode demanded mode
    /// @return Pointer to dynamically alocated ControllerLoop
    static ControllerLoop* ControllerLoopFactory(ControllerMode mode);

protected:
    const ControllerMode _mode;
    std::vector<std::string> required_controllers;

    /// @brief Check if joystick input vector is correct
    /// @param joystick joystick axes deflaction
    /// @param minimalSize minimal length of deflation vector that can be interpreted
    /// @return return true if joystick input vector is long enough
    bool checkJoystickLength(const Eigen::VectorXd& joystick, const int minimalSize);
};