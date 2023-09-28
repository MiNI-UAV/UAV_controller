#include "controller_loop_QANGLE.hpp"
#include "../../utils.hpp"

ControllerLoopQANGLE::ControllerLoopQANGLE():
    ControllerLoop(ControllerMode::QANGLE)
{
    required_pids.assign({"Roll", "Pitch", "Yaw", "W", "Z", "Fi", "Theta", "Psi"});
}

void ControllerLoopQANGLE::job(
    std::map<std::string,PID>& pids,
    Control& control,
    NS& navisys
) 
{
    Eigen::Vector3d pos = navisys.getPosition();
    Eigen::Vector3d vel = navisys.getLinearVelocity();
    Eigen::Vector3d ori = navisys.getOrientation();
    Eigen::Vector3d angVel = navisys.getAngularVelocity();

    double demandedW = pids.at("Z").calc(demandedZ - pos(2));
    double demandedP = pids.at("Fi").calc(circularError(demandedFi, ori(0)));
    double demandedQ = pids.at("Theta").calc(circularError(demandedTheta, ori(1)));
    double demandedR = pids.at("Psi").calc(circularError(demandedPsi, ori(2)));

    double climb_rate = pids.at("W").calc(demandedW-vel(2));
    double roll_rate = pids.at("Roll").calc(demandedP-angVel(0));
    double pitch_rate = pids.at("Pitch").calc(demandedQ-angVel(1));
    double yaw_rate = pids.at("Yaw").calc(demandedR-angVel(2));

    Eigen::VectorXd vec = applyMixerRotors(climb_rate,roll_rate,pitch_rate,yaw_rate);
    control.sendSpeed(vec);
}

void ControllerLoopQANGLE::handleJoystick(Eigen::Vector4d joystick) 
{
    constexpr double angleLimit = std::numbers::pi/5.0;
    demandedZ -= joystick[1]/10.0;
    demandedFi = joystick[2]*angleLimit;
    demandedTheta = -joystick[3]*angleLimit;
    demandedPsi = clampAngle(demandedPsi + joystick[0]/30.0);
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
