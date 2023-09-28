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
    pids = params->pids;
    for(auto& elem : pids)
    {
        elem.second.set_dt(def::STEP_TIME);
    }
    setMode(ControllerModeFromString(params->initialMode.data()));
    syncWithPhysicEngine(ctx,uav_address);
    startLoop();
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
                startLoop();
                status = Status::running;
            break;
        }
    }
}

void Controller::startLoop()
{
    loop.emplace(std::round(def::STEP_TIME*1000.0),[this] () 
    {
        if(controller_loop == nullptr) return;
        controller_loop->job(
            pids,
            control,
            navisys
        );
    }
    ,status
    );
}

void Controller::syncWithPhysicEngine(zmq::context_t *ctx, std::string uav_address)
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

void Controller::setMode(ControllerMode new_mode)
{
    auto new_loop = ControllerLoop::ControllerLoopFactory(new_mode);
    for (auto& pid_name: new_loop->requiredPIDs())
    {
        if(!pids.contains(pid_name))
        {
            std::cerr << "Missing pid " << pid_name << " to run "
                << ControllerModeToString(new_mode) << " mode" << std::endl;
            return;
        }
    }
    //TODO: remove below loop after changes
    for(auto pid: UAVparams::getSingleton()->pids)
    {
        pid.second.clear();
    }
    new_loop->overridePosition(navisys.getPosition(),navisys.getOrientation());
    std::swap(new_loop,controller_loop);
    if(new_loop != nullptr) delete new_loop;
    status = Status::reload;
}

void Controller::exitController()
{
    status = Status::exiting;
}
