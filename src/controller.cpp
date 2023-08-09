#include "controller.hpp"
#include <iostream>
#include "NS.hpp"
#include "NS2.hpp"

Controller::Controller(zmq::context_t *ctx, std::string uav_address, Params& _params):
state(ctx, uav_address, mode,[this](ControllerMode mode){setMode(mode);},[this](){exitController();}),
env(ctx, uav_address),
//navisys(ctx, uav_address),
navisys2(env),
control(ctx, uav_address),
params{_params}
{
    status = Status::running;
    mode = ControllerMode::position;
    jobs[ControllerMode::none] = [this](){
        Eigen::VectorXd vec = params.mixer(0.0,0.0,0.0,0.0);
        control.sendSpeed(vec);
    };
    jobs[ControllerMode::angle] = [&](){angleControllLoop();};
    jobs[ControllerMode::acro] = [&](){acroControllLoop();};
    jobs[ControllerMode::position] = [&](){positionControllLoop();};
    loop.emplace(std::round(step_time),jobs[mode],status);
    syncWithPhysicEngine(ctx,uav_address);
}

Controller::~Controller()
{
    std::cout << "Exiting controller!" << std::endl;
}

void Controller::run()
{
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
    auto pos = navisys2.getPosition();
    state.demandedX = pos(0);
    state.demandedY = pos(1);
    state.demandedZ = pos(2);
    auto ori = navisys2.getOrientation();
    state.demandedFi = ori(0);
    state.demandedTheta = ori(1);
    state.demandedPsi = ori(2);
    
}

void Controller::acroControllLoop()
{
    Eigen::Vector3d angVel = navisys2.getAngularVelocity();

    double climb_rate = (state.throttle+1.0)*params.hoverRotorSpeed;
    double roll_rate = params.pids.at("Roll").calc(state.demandedP-angVel(0));
    double pitch_rate = params.pids.at("Pitch").calc(state.demandedQ-angVel(1));
    double yaw_rate = params.pids.at("Yaw").calc(state.demandedR-angVel(2));

    Eigen::VectorXd vec = params.mixer(climb_rate,roll_rate,pitch_rate,yaw_rate);
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
    Eigen::Vector3d pos = navisys2.getPosition();
    Eigen::Vector3d vel = navisys2.getLinearVelocity();
    Eigen::Vector3d ori = navisys2.getOrientation();
    Eigen::Vector3d angVel = navisys2.getAngularVelocity();

    std::cout << vel << std::endl  << std::endl;

    double demandedW = params.pids.at("Z").calc(state.demandedZ - pos(2));
    double demandedP = params.pids.at("Fi").calc(circularError(state.demandedFi, ori(0)));
    double demandedQ = params.pids.at("Theta").calc(circularError(state.demandedTheta, ori(1)));
    double demandedR = params.pids.at("Psi").calc(circularError(state.demandedPsi, ori(2)));

    double climb_rate = params.pids.at("W").calc(demandedW-vel(2));
    double roll_rate = params.pids.at("Roll").calc(demandedP-angVel(0));
    double pitch_rate = params.pids.at("Pitch").calc(demandedQ-angVel(1));
    double yaw_rate = params.pids.at("Yaw").calc(demandedR-angVel(2));

    Eigen::VectorXd vec = params.mixer(climb_rate,roll_rate,pitch_rate,yaw_rate);
    control.sendSpeed(vec);
}

void Controller::positionControllLoop()
{
    Eigen::Vector3d pos = navisys2.getPosition();
    Eigen::Vector3d vel = navisys2.getLinearVelocity();
    Eigen::Vector3d ori = navisys2.getOrientation();
    Eigen::Vector3d angVel = navisys2.getAngularVelocity();

    double demandedU = params.pids.at("X").calc(state.demandedX - pos(0));
    double demandedV = params.pids.at("Y").calc(state.demandedY - pos(1));
    
    double demandedFi_star = params.pids.at("V").calc(demandedV - vel(1));
    double demandedTheta_star = params.pids.at("U").calc(demandedU - vel(0));

    double PsiCos = std::cos(ori(2));
    double PsiSin = std::sin(ori(2));
    double demandedFi = demandedFi_star*PsiCos + demandedTheta_star*PsiSin;
    double demandedTheta = - demandedFi_star*PsiSin + demandedTheta_star*PsiCos;

    double demandedP = params.pids.at("Fi").calc(demandedFi - ori(0));
    double demandedQ = params.pids.at("Theta").calc(demandedTheta - ori(1));

    double demandedW = params.pids.at("Z").calc(state.demandedZ - pos(2));
    double demandedR = params.pids.at("Psi").calc(circularError(state.demandedPsi, ori(2)));

    double climb_rate = params.pids.at("W").calc(demandedW-vel(2));
    double roll_rate = params.pids.at("Roll").calc(demandedP-angVel(0));
    double pitch_rate = params.pids.at("Pitch").calc(demandedQ-angVel(1));
    double yaw_rate = params.pids.at("Yaw").calc(demandedR-angVel(2));

    Eigen::VectorXd vec = params.mixer(climb_rate,roll_rate,pitch_rate,yaw_rate);
    control.sendSpeed(vec);
}



void Controller::setMode(ControllerMode new_mode)
{
    for(auto pid: params.pids)
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
