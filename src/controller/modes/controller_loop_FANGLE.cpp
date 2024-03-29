#include "controller_loop_FANGLE.hpp"
#include "../../utils.hpp"

ControllerLoopFANGLE::ControllerLoopFANGLE():
    ControllerLoop(ControllerMode::FANGLE)
{
    required_controllers.assign({"Roll", "Pitch", "Yaw", "U", "Fi", "Theta", "Psi"});
}

void ControllerLoopFANGLE::job(
    std::map<std::string,std::unique_ptr<Controller>>& controllers,
    Control& control,
    NS& navisys
) 
{
    Eigen::Vector3d vel = navisys.getRotationMatrixBodyToWorld().transpose() *  navisys.getLinearVelocity();
    Eigen::Vector3d ori = navisys.getOrientation();
    Eigen::Vector3d angVel = navisys.getAngularVelocity();

    double demandedP = controllers.at("Fi")->calc(circularError(demandedFi, ori(0)), 0.0);
    double demandedQ = controllers.at("Theta")->calc(circularError(demandedTheta, ori(1)), 0.0);

    double throttle = controllers.at("U")->calc(demandedVx, vel(0));
    double roll_rate = controllers.at("Roll")->calc(demandedP, angVel(0));
    double pitch_rate = controllers.at("Pitch")->calc(demandedQ ,angVel(1));
    double yaw_rate = 0.0;

    // Disable rudder when plane tilted
    if(std::abs(ori(0)) > angleLimit/2 )
    {
        demandedPsi = ori(2);
        yaw_rate = controllers.at("Yaw")->calc(demanded_R, angVel(2));
    }
    else
    {
        double demandedR = controllers.at("Psi")->calc(circularError(demandedPsi, ori(2)), 0.0);
        yaw_rate = controllers.at("Yaw")->calc(demandedR, angVel(2));
    }

    Eigen::VectorXd vec = applyMixerRotorsHover(throttle,roll_rate,pitch_rate,yaw_rate);
    control.sendSpeed(vec);
    Eigen::VectorXd surf = applyMixerSurfaces(throttle,roll_rate,pitch_rate,yaw_rate);
    control.sendSurface(surf);
}

void ControllerLoopFANGLE::handleJoystick(Eigen::VectorXd joystick) 
{
    if(!checkJoystickLength(joystick,4)) return;
    demandedVx += joystick[0]/2.0;
    if(demandedVx < 0.0) demandedVx = 0.0;
    demandedFi = joystick[1]*angleLimit;
    demandedTheta = -joystick[2]*angleLimit;
    demandedPsi = clampAngle(demandedPsi + joystick[3]/50.0);
    demanded_R = joystick[3]*3.0;
}

std::string ControllerLoopFANGLE::demandInfo() 
{
    std::stringstream ss;
    std::string s;
    ss.precision(3);
    ss << std::fixed << ControllerModeToString(_mode) << ",";
    ss << demandedFi << "," << demandedTheta << "," << demandedPsi << "," << demandedVx;
    return ss.str();
}

void ControllerLoopFANGLE::overridePositionAndSpeed(
    [[maybe_unused]] Eigen::Vector3d position,
    [[maybe_unused]] Eigen::Vector3d orientation,
    [[maybe_unused]] Eigen::Vector3d velocity
) 
{
    demandedVx = velocity.x();
    demandedPsi = orientation.z();
}
