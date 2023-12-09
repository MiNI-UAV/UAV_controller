#include "controller_loop_QANGLE.hpp"
#include "../../utils.hpp"

ControllerLoopQANGLE::ControllerLoopQANGLE():
    ControllerLoop(ControllerMode::QANGLE)
{
    required_controllers.assign({"Roll", "Pitch", "Yaw", "W", "Z", "Fi", "Theta", "Psi"});
}

void ControllerLoopQANGLE::job(
    std::map<std::string,std::unique_ptr<Controller>>& controllers,
    Control& control,
    NS& navisys
) 
{
    Eigen::Vector3d pos = navisys.getPosition();
    Eigen::Vector3d vel = navisys.getLinearVelocity();
    Eigen::Vector3d ori = navisys.getOrientation();
    Eigen::Vector3d angVel = navisys.getAngularVelocity();

    double demandedW = controllers.at("Z")->calc(demandedZ, pos(2));
    double demandedP = controllers.at("Fi")->calc(circularError(demandedFi, ori(0)), 0.0);
    double demandedQ = controllers.at("Theta")->calc(circularError(demandedTheta, ori(1)), 0.0);
    double demandedR = controllers.at("Psi")->calc(circularError(demandedPsi, ori(2)), 0.0);

    double climb_rate = controllers.at("W")->calc(demandedW, vel(2));
    double roll_rate = controllers.at("Roll")->calc(demandedP, angVel(0));
    double pitch_rate = controllers.at("Pitch")->calc(demandedQ, angVel(1));
    double yaw_rate = controllers.at("Yaw")->calc(demandedR, angVel(2));

    Eigen::VectorXd vec = applyMixerRotors(climb_rate,roll_rate,pitch_rate,yaw_rate);
    control.sendSpeed(vec);
}

void ControllerLoopQANGLE::handleJoystick(Eigen::VectorXd joystick) 
{
    constexpr double angleLimit = std::numbers::pi/5.0;
    if(!checkJoystickLength(joystick,4)) return;
    demandedZ -= joystick[0]/10.0;
    demandedFi = joystick[1]*angleLimit;
    demandedTheta = -joystick[2]*angleLimit;
    demandedPsi = clampAngle(demandedPsi + joystick[3]/30.0);
}

std::string ControllerLoopQANGLE::demandInfo() 
{
    std::stringstream ss;
    std::string s;
    ss.precision(3);
    ss << std::fixed << ControllerModeToString(_mode) << ",";
    ss << demandedFi << "," << demandedTheta << "," << demandedPsi << "," << demandedZ;
    return ss.str();
}

void ControllerLoopQANGLE::overridePosition(
    Eigen::Vector3d position,
    Eigen::Vector3d orientation
) 
{
    demandedZ = position.z();
    demandedPsi = orientation.z();
}
