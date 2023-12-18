#include "controller_loop_RGUIDED.hpp"
#include "../../utils.hpp"
#include "common.hpp"

ControllerLoopRGUIDED::ControllerLoopRGUIDED():
    ControllerLoop(ControllerMode::RGUIDED), target{UAVparams::getSingleton()->target}
{
    required_controllers.assign({"H", "V", "Theta", "Psi"});
}

void ControllerLoopRGUIDED::job(
    [[maybe_unused]] std::map<std::string,std::unique_ptr<Controller>>& controllers,
    Control& control,
    [[maybe_unused]] NS& navisys
) 
{
    Eigen::Vector3d pos = navisys.getPosition();
    Eigen::Vector3d ori = navisys.getOrientation();
    Eigen::Vector3d vel = navisys.getLinearVelocity();
    Eigen::Vector3d angVel = navisys.getAngularVelocity();

    Eigen::Vector3d target_heading = target - pos;

    if(target_heading.norm() < 10.0)
    {
        std::cout << "Target reached" << std::endl;
        Eigen::VectorXd surf = applyMixerSurfaces(0.0, 0.0, 0.0, 0.0);
        control.sendSurface(surf);
        control.setMode(ControllerMode::RMANUAL);
    }
    double norm_2d = std::sqrt(std::pow(target_heading.x(),2.0) + std::pow(target_heading.y(),2.0));
    double demandedTheta, demandedPsi;

    if(norm_2d > 0.1)
    {
        demandedTheta = std::atan2(-target_heading.z(), norm_2d);
        demandedPsi = std::atan2(target_heading.y(), target_heading.x());
    }
    else
    {
        if(target_heading.z() > 0.0)
        {
            demandedTheta = std::numbers::pi/2.0;
            demandedPsi = ori.z();
        }
        else
        {
            demandedTheta = -std::numbers::pi/2.0;
            demandedPsi = ori.z();
        }
    }

    if(std::abs(circularError(demandedTheta, ori(1))) > detection_limit 
        || std::abs(circularError(demandedPsi, ori(2)) > detection_limit))
    {
        std::cout << "Target lost" << std::endl;
        Eigen::VectorXd surf = applyMixerSurfaces(0.0, 0.0, 0.0, 0.0);
        control.sendSurface(surf);
        control.setMode(ControllerMode::RMANUAL);
    }

    double vel_norm_2d = std::sqrt(std::pow(vel.x(),2.0) + std::pow(vel.y(),2.0));
    double vel_theta, vel_psi;
    if(vel_norm_2d > 0.1)
    {
        vel_theta = std::atan2(-vel.z(), vel_norm_2d);
        vel_psi = std::atan2(vel.y(), vel.x());
    }
    else
    {
        if(vel.z() > 0.0)
        {
            vel_theta = std::numbers::pi/2.0;
            vel_psi = ori.z();
        }
        else
        {
            vel_theta = -std::numbers::pi/2.0;
            vel_psi = ori.z();
        }
    }


    double demanded_V = controllers.at("Theta")->calc(circularError(demandedTheta, vel_theta), 0.0);
    double demanded_H = controllers.at("Psi")->calc(circularError(demandedPsi, vel_psi), 0.0);

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

std::string ControllerLoopRGUIDED::demandInfo() 
{
    std::stringstream ss;
    std::string s;
    ss.precision(3);
    ss << std::fixed << ControllerModeToString(_mode) << ",";
    ss << target.x() << "," << target.y() << "," << target.z();
    return ss.str();
}
