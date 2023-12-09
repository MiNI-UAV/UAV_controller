#include "controller.hpp"
#include <iostream>
#include "../defines.hpp"

ControlSystem::ControlSystem(
    zmq::context_t *ctx,
    std::string uav_address
    ):
controller_loop{ControllerLoop::ControllerLoopFactory(ControllerMode::NONE)},
control{new Control(ctx, uav_address,this)},
env(ctx, uav_address),
navisys(env)
{
    const UAVparams* params = UAVparams::getSingleton();
    status = Status::running;
    for(const auto& [key, value]: params->controllers)
    {
        controllers.insert(std::make_pair(key, std::move(value->clone())));
    }
    for(auto& [key, value]: controllers)
    {
        value->set_dt(def::STEP_TIME);
    }
    setMode(ControllerModeFromString(params->initialMode.data()));
    syncWithPhysicEngine(ctx,uav_address);
    startLoop();
    std::cout << "Constructing controller done" << std::endl;
}

ControlSystem::~ControlSystem()
{
    delete controller_loop;
    delete control;
    std::cout << "Exiting controller!" << std::endl;
}

void ControlSystem::run()
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
                control->start();
                std::cout << "Running in " << ControllerModeToString(controller_loop->getMode()) << " mode" << std::endl;
                loop->go();
                control->recv();
            break;
            case Status::exiting:
                std::cout << "Exiting..." << std::endl;
                control->stop();
                run = false;
            break;
            case Status::reload:
                startLoop();
                status = Status::running;
            break;
        }
    }
}

void ControlSystem::startLoop()
{
    loop.emplace(std::round(def::STEP_TIME*1000.0),[this] () 
    {
        if(controller_loop == nullptr) return;
        controller_loop->job(
            controllers,
            *control,
            navisys
        );
    }
    ,status
    );
}

void ControlSystem::syncWithPhysicEngine(zmq::context_t *ctx, std::string uav_address)
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
	control->prepare();
	std::cout << "Synchronized!" << std::endl;
}

void ControlSystem::setMode(ControllerMode new_mode)
{
    auto new_loop = ControllerLoop::ControllerLoopFactory(new_mode);
    for (auto& Controller_name: new_loop->requiredcontrollers())
    {
        if(!controllers.contains(Controller_name))
        {
            std::cerr << "Missing Controller " << Controller_name << " to run "
                << ControllerModeToString(new_mode) << " mode" << std::endl;
            return;
        }
    }
    new_loop->overridePosition(navisys.getPosition(),navisys.getOrientation());
    std::swap(new_loop,controller_loop);
    if(new_loop != nullptr) delete new_loop;
    status = Status::reload;
}

void ControlSystem::exitController()
{
    status = Status::exiting;
}
