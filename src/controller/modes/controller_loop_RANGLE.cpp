#include "controller_loop_RANGLE.hpp"
#include "../../utils.hpp"

ControllerLoopRANGLE::ControllerLoopRANGLE():
    ControllerLoop(ControllerMode::RANGLE)
{
    required_controllers.assign({"H", "V", "Theta", "Psi"});
}

void ControllerLoopRANGLE::job(
    [[maybe_unused]] std::map<std::string,std::unique_ptr<Controller>>& controllers,
    Control& control,
    [[maybe_unused]] NS& navisys
) 
{
    Eigen::Vector3d ori = navisys.getOrientation();
    Eigen::Vector3d angVel = navisys.getAngularVelocity();

    double demanded_V = controllers.at("Theta")->calc(circularError(demandedTheta, ori(1)), 0.0);
    double demanded_H = controllers.at("Psi")->calc(circularError(demandedPsi, ori(2)), 0.0);

    double V_rate = controllers.at("V")->calc(demanded_V,angVel(1));
    double H_rate = controllers.at("H")->calc(demanded_H,angVel(2));

    if(std::abs(angVel(0)) < 3.0)
    {
        Eigen::VectorXd surf = applyMixerSurfaces(0.0, 0.0, 0.0, 0.0);
        control.sendSurface(surf);
        return;
    }
    double est_roll = ori(0);
    double rot_pitch = V_rate * cos(est_roll) + H_rate * sin(est_roll);
    Eigen::VectorXd surf = applyMixerSurfaces(0.0, 0.0, rot_pitch, 0.0);
    control.sendSurface(surf);
}

void ControllerLoopRANGLE::handleJoystick(Eigen::VectorXd joystick) 
{
    if(!checkJoystickLength(joystick,4)) return;
    demandedTheta = -joystick[2]*angleLimit;
    demandedPsi = clampAngle(demandedPsi + joystick[1]/30.0);
}

std::string ControllerLoopRANGLE::demandInfo() 
{
    std::stringstream ss;
    std::string s;
    ss.precision(3);
    ss << std::fixed << ControllerModeToString(_mode) << ",";
    ss << demandedTheta << "," << demandedPsi;
    return ss.str();
}

void ControllerLoopRANGLE::overridePositionAndSpeed(
    [[maybe_unused]] Eigen::Vector3d position,
    [[maybe_unused]] Eigen::Vector3d orientation,
    [[maybe_unused]] Eigen::Vector3d velocity
) 
{
    demandedPsi = orientation.z();
}
