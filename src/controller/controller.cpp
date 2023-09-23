#include "controller.hpp"
#include <iostream>
#include "../defines.hpp"

Controller::Controller(
    zmq::context_t *ctx,
    std::string uav_address
    ):
controller_loop{ControllerLoop::ControllerLoopFactory(ControllerMode::NONE)},
state{new State(
    ctx,
    uav_address,
    this
    )},
env(ctx, uav_address),
navisys(env),
control(ctx, uav_address)
{
    const UAVparams* params = UAVparams::getSingleton();
    status = Status::running;
    loop.emplace(std::round(def::STEP_TIME*1000.0),[this]() {controller_loop->job(
        state,
        pids,
        control,
        navisys
    );},status);
    pids = params->pids;
    for(auto& elem : pids)
    {
        elem.second.set_dt(def::STEP_TIME);
    }
    setMode(ControllerModeFromString(params->initialMode.data()));
    syncWithPhysicEngine(ctx,uav_address);
    std::cout << "Constructing controller done" << std::endl;
}

Controller::~Controller()
{
    delete controller_loop;
    delete state;
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
                std::cout << "Running in " << ControllerModeToString(controller_loop->getMode()) << " mode" << std::endl;
                loop->go();
                control.recv();
            break;
            case Status::exiting:
                std::cout << "Exiting..." << std::endl;
                control.stop();
                run = false;
            break;
            case Status::reload:
                loop.emplace(std::round(def::STEP_TIME*1000.0),[this] () {controller_loop->job(
                    state,
                    pids,
                    control,
                    navisys
                );},status);
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
    state->demandedX = pos(0);
    state->demandedY = pos(1);
    state->demandedZ = pos(2);
    auto ori = navisys.getOrientation();
    state->demandedFi = ori(0);
    state->demandedTheta = ori(1);
    state->demandedPsi = ori(2);
    
}

void Controller::setMode(ControllerMode new_mode)
{
    for(auto pid: UAVparams::getSingleton()->pids)
    {
        pid.second.clear();
    }
    setCurrentDemands();
    if(controller_loop != nullptr) delete controller_loop;
    controller_loop = ControllerLoop::ControllerLoopFactory(new_mode);
    status = Status::reload;
}

void Controller::exitController()
{
    status = Status::exiting;
}
