#include "controller.hpp"
#include <iostream>
#include "UAV_NS/NS.hpp"
#include "defines.hpp"

Controller::Controller(zmq::context_t *ctx, std::string uav_address):
state(ctx, uav_address, mode,[this](ControllerMode mode){setMode(mode);},[this](){exitController();}),
env(ctx, uav_address),
navisys(env),
control(ctx, uav_address)
{
    const UAVparams* params = UAVparams::getSingleton();
    status = Status::running;
    mode = ControllerMode::position;
    jobs[ControllerMode::none] = [this](){
        Eigen::VectorXd vec = applyMixerRotors(0.0,0.0,0.0,0.0);
        control.sendSpeed(vec);
    };
    jobs[ControllerMode::angle] = [&](){angleControllLoop();};
    jobs[ControllerMode::acro] = [&](){acroControllLoop();};
    jobs[ControllerMode::position] = [&](){positionControllLoop();};
    loop.emplace(std::round(step_time),jobs[mode],status);
    pids = params->pids;
    syncWithPhysicEngine(ctx,uav_address);
    std::cout << "Constructing controller done" << std::endl;
}

Controller::~Controller()
{
    std::cout << "Exiting controller!" << std::endl;
}

void Controller::run()
{
    std::cout << "Initializing controller" << std::endl;
    bool run = true;
    
    while(run)
    {
        switch(status)
        {
            case Status::idle:
                //TODO
            break;
            case Status::running:
                control.start();
                std::cout << "Running in " << ToString(mode) << " mode" << std::endl;
                loop->go();
                control.recv();
            break;
            case Status::exiting:
                std::cout << "Exiting..." << std::endl;
                control.stop();
                run = false;
            break;
            case Status::reload:
                loop.emplace(std::round(step_time),jobs[mode],status);
                status = Status::running;
            break;
        }
    }
}

void Controller::syncWithPhysicEngine(zmq::context_t *ctx,std::string uav_address)
{
    std::cout << "Attempting to sync..." << std::endl;
	zmq::socket_t sock = zmq::socket_t(*ctx, zmq::socket_type::sub);
	sock.set(zmq::sockopt::subscribe, "idle");
	sock.connect(uav_address + "/state");
	while (1)
	{
		zmq::message_t msg;
		const auto res = sock.recv(msg, zmq::recv_flags::none);
		if (!res)
		{
			std::cerr << "Sync error" << std::endl;
			exit(1);
		}
		if(std::string(static_cast<char*>(msg.data()), msg.size()).compare("idle") == 0) break;
	}
	sock.close();
	control.prepare();
	std::cout << "Synchronized!" << std::endl;
}

void Controller::setCurrentDemands()
{
    auto pos = navisys.getPosition();
    state.demandedX = pos(0);
    state.demandedY = pos(1);
    state.demandedZ = pos(2);
    auto ori = navisys.getOrientation();
    state.demandedFi = ori(0);
    state.demandedTheta = ori(1);
    state.demandedPsi = ori(2);
    
}

void Controller::acroControllLoop()
{
    Eigen::Vector3d angVel = navisys.getAngularVelocity();
    
    double roll_rate = pids.at("Roll").calc(step_time,state.demandedP-angVel(0));
    double pitch_rate = pids.at("Pitch").calc(step_time,state.demandedQ-angVel(1));
    double yaw_rate = pids.at("Yaw").calc(step_time,state.demandedR-angVel(2));

    Eigen::VectorXd vec = applyMixerRotorsHover(state.throttle,roll_rate,pitch_rate,yaw_rate);
    control.sendSpeed(vec);
}

double circularError(double demanded, double val)
{
    double diff = demanded-val;
    if(diff > std::numbers::pi) return -2*std::numbers::pi + diff;
    if(diff < -std::numbers::pi) return +2*std::numbers::pi + diff;
    return diff;
}

void Controller::angleControllLoop()
{
    Eigen::Vector3d pos = navisys.getPosition();
    Eigen::Vector3d vel = navisys.getLinearVelocity();
    Eigen::Vector3d ori = navisys.getOrientation();
    Eigen::Vector3d angVel = navisys.getAngularVelocity();

    double demandedW = pids.at("Z").calc(step_time,state.demandedZ - pos(2));
    double demandedP = pids.at("Fi").calc(step_time,circularError(state.demandedFi, ori(0)));
    double demandedQ = pids.at("Theta").calc(step_time,circularError(state.demandedTheta, ori(1)));
    double demandedR = pids.at("Psi").calc(step_time,circularError(state.demandedPsi, ori(2)));

    double climb_rate = pids.at("W").calc(step_time,demandedW-vel(2));
    double roll_rate = pids.at("Roll").calc(step_time,demandedP-angVel(0));
    double pitch_rate = pids.at("Pitch").calc(step_time,demandedQ-angVel(1));
    double yaw_rate = pids.at("Yaw").calc(step_time,demandedR-angVel(2));

    Eigen::VectorXd vec = applyMixerRotors(climb_rate,roll_rate,pitch_rate,yaw_rate);
    control.sendSpeed(vec);
}

void Controller::positionControllLoop()
{
    Eigen::Vector3d pos = navisys.getPosition();
    Eigen::Vector3d vel = navisys.getLinearVelocity();
    Eigen::Vector3d ori = navisys.getOrientation();
    Eigen::Vector3d angVel = navisys.getAngularVelocity();

    double demandedU = pids.at("X").calc(step_time,state.demandedX - pos(0));
    double demandedV = pids.at("Y").calc(step_time,state.demandedY - pos(1));
    
    double demandedFi_star = pids.at("V").calc(step_time,demandedV - vel(1));
    double demandedTheta_star = pids.at("U").calc(step_time,demandedU - vel(0));

    double PsiCos = std::cos(ori(2));
    double PsiSin = std::sin(ori(2));
    double demandedFi = demandedFi_star*PsiCos + demandedTheta_star*PsiSin;
    double demandedTheta = - demandedFi_star*PsiSin + demandedTheta_star*PsiCos;

    double demandedP = pids.at("Fi").calc(step_time,demandedFi - ori(0));
    double demandedQ = pids.at("Theta").calc(step_time,demandedTheta - ori(1));

    double demandedW = pids.at("Z").calc(step_time,state.demandedZ - pos(2));
    double demandedR = pids.at("Psi").calc(step_time,circularError(state.demandedPsi, ori(2)));

    double climb_rate = pids.at("W").calc(step_time,demandedW-vel(2));
    double roll_rate = pids.at("Roll").calc(step_time,demandedP-angVel(0));
    double pitch_rate = pids.at("Pitch").calc(step_time,demandedQ-angVel(1));
    double yaw_rate = pids.at("Yaw").calc(step_time,demandedR-angVel(2));

    Eigen::VectorXd vec = applyMixerRotors(climb_rate,roll_rate,pitch_rate,yaw_rate);
    control.sendSpeed(vec);
}



void Controller::setMode(ControllerMode new_mode)
{
    for(auto pid: UAVparams::getSingleton()->pids)
    {
        pid.second.clear();
    }
    setCurrentDemands();
    mode = new_mode;
    status = Status::reload;
}

void Controller::exitController()
{
    status = Status::exiting;
}
