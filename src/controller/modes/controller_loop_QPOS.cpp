#include "controller_loop_QPOS.hpp"
#include "../../utils.hpp"

ControllerLoopQPOS::ControllerLoopQPOS():
    ControllerLoop(ControllerMode::QPOS)
{
    required_controllers.assign({"Roll", "Pitch", "Yaw", "W", "Z", "Fi",
        "Theta", "Psi", "X", "Y", "V", "U"});
}

void ControllerLoopQPOS::job(
    std::map<std::string,std::unique_ptr<Controller>>& controllers,
    Control& control,
    NS& navisys
) 
{
    Eigen::Vector3d pos = navisys.getPosition();
    Eigen::Vector3d vel = navisys.getLinearVelocity();
    Eigen::Vector3d ori = navisys.getOrientation();
    Eigen::Vector3d angVel = navisys.getAngularVelocity();

    double demandedU = controllers.at("X")->calc(demandedX, pos(0));
    double demandedV = controllers.at("Y")->calc(demandedY, pos(1));
    
    double demandedFi_star = controllers.at("V")->calc(demandedV, vel(1));
    double demandedTheta_star = controllers.at("U")->calc(demandedU, vel(0));

    double PsiCos = std::cos(ori(2));
    double PsiSin = std::sin(ori(2));
    double demandedFi = demandedFi_star*PsiCos + demandedTheta_star*PsiSin;
    double demandedTheta = - demandedFi_star*PsiSin + demandedTheta_star*PsiCos;

    double demandedP = controllers.at("Fi")->calc(demandedFi, ori(0));
    double demandedQ = controllers.at("Theta")->calc(demandedTheta, ori(1));

    double demandedW = controllers.at("Z")->calc(demandedZ, pos(2));
    double demandedR = controllers.at("Psi")->calc(circularError(demandedPsi, ori(2)), 0.0);

    double climb_rate = controllers.at("W")->calc(demandedW, vel(2));
    double roll_rate = controllers.at("Roll")->calc(demandedP, angVel(0));
    double pitch_rate = controllers.at("Pitch")->calc(demandedQ, angVel(1));
    double yaw_rate = controllers.at("Yaw")->calc(demandedR, angVel(2));

    Eigen::VectorXd vec = applyMixerRotors(climb_rate,roll_rate,pitch_rate,yaw_rate);
    control.sendSpeed(vec);
}

void ControllerLoopQPOS::handleJoystick(Eigen::VectorXd joystick) 
{
    constexpr double angleLimit = std::numbers::pi/5.0;
    if(!checkJoystickLength(joystick,4)) return;
    demandedZ -= joystick[0]/10.0;
    demandedPsi = clampAngle(demandedPsi + joystick[3]/20.0);
    demandedX += ((joystick[2]*angleLimit)*std::cos(demandedPsi) - (joystick[1]*angleLimit)*std::sin(demandedPsi))/2.0;
    demandedY += ((joystick[2]*angleLimit)*std::sin(demandedPsi) + (joystick[1]*angleLimit)*std::cos(demandedPsi))/2.0;
}

std::string ControllerLoopQPOS::demandInfo() {
    std::stringstream ss;
    std::string s;
    ss.precision(3);
    ss << std::fixed << ControllerModeToString(_mode) << ",";
    ss << demandedX << "," << demandedY << "," << demandedZ << "," << demandedPsi;
    return ss.str();
}

void ControllerLoopQPOS::overridePositionAndSpeed(
    [[maybe_unused]] Eigen::Vector3d position,
    [[maybe_unused]] Eigen::Vector3d orientation,
    [[maybe_unused]] Eigen::Vector3d velocity
) 
{
    demandedX = position.x();
    demandedY = position.y();
    demandedZ = position.z();
    demandedPsi = orientation.z();
}
