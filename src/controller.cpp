#include "controller.hpp"
#include <iostream>

Controller::Controller(zmq::context_t *ctx, std::string uav_address):
state(ctx, uav_address, mode,[this](ControllerMode mode){setMode(mode);},[this](){exitController();}),
gps(ctx, uav_address),
gyro(ctx, uav_address),
control(ctx, uav_address)
{
    status = Status::running;
    loadPIDs("");
    mode = ControllerMode::angle;
    mixer = [&](double c, double r, double p, double y) {return controlMixer4(c,r,p,y,maxRotorSpeed);};
    jobs[ControllerMode::none] = [](){};
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

void Controller::loadPIDs(std::string configPath)
{
    if(configPath.empty())
    {
        pids.insert(std::make_pair("Z",PID(step_time / 1000.0, 2.122, 0.035, -0.387, -1000, 1000)));
        pids.insert(std::make_pair("Fi",PID(step_time / 1000.0, 9.584, 0.798, 0.192, -1000, 1000)));
        pids.insert(std::make_pair("Theta",PID(step_time / 1000.0, 5.191, 0.228, 0.127, -1000, 1000)));
        pids.insert(std::make_pair("Psi",PID(step_time / 1000.0, 5.288, 0.230, -0.151, -1000, 1000)));

        pids.insert(std::make_pair("W",PID(step_time / 1000.0, -3556.149, -538.572, -112.917, 0, 1000)));
        pids.insert(std::make_pair("Roll",PID(step_time / 1000.0, -6.249, -0.904, -0.219, -250, 250)));
        pids.insert(std::make_pair("Pitch",PID(step_time / 1000.0, 6.304, 1.174, 0.433, -250, 250)));
        pids.insert(std::make_pair("Yaw",PID(step_time / 1000.0, 112.662, 22.778, 3.419, -250, 250)));
    }
}

void Controller::setCurrentDemands()
{
    auto pos = gps.getGPSPos();
    state.demandedX = pos(0);
    state.demandedY = pos(1);
    state.demandedZ = pos(2);
    auto ori = gps.getAH();
    state.demandedFi = ori(0);
    state.demandedTheta = ori(1);
    state.demandedPsi = ori(2);
    
}

void Controller::acroControllLoop()
{
    Eigen::Vector3d angVel = gyro.getAngularVel();

    double climb_rate = (state.throttle+1.0)*hoverRotorSpeed;
    double roll_rate = pids.at("Roll").calc(state.demandedP-angVel(0));
    double pitch_rate = pids.at("Pitch").calc(state.demandedQ-angVel(1));
    double yaw_rate = pids.at("Yaw").calc(state.demandedR-angVel(2));
    Eigen::VectorXd vec = mixer(climb_rate,roll_rate,pitch_rate,yaw_rate);
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
    Eigen::Vector3d pos = gps.getGPSPos();
    Eigen::Vector3d vel = gps.getGPSVel();
    Eigen::Vector3d ori = gps.getAH();
    Eigen::Vector3d angVel = gyro.getAngularVel();

    double demandedW = pids.at("Z").calc(state.demandedZ - pos(2));
    double demandedP = pids.at("Fi").calc(circularError(state.demandedFi, ori(0)));
    double demandedQ = pids.at("Theta").calc(circularError(state.demandedTheta, ori(1)));
    double demandedR = pids.at("Psi").calc(circularError(state.demandedPsi, ori(2)));

    double climb_rate = pids.at("W").calc(demandedW-vel(2));
    double roll_rate = pids.at("Roll").calc(demandedP-angVel(0));
    double pitch_rate = pids.at("Pitch").calc(demandedQ-angVel(1));
    double yaw_rate = pids.at("Yaw").calc(demandedR-angVel(2));
    Eigen::VectorXd vec = mixer(climb_rate,roll_rate,pitch_rate,yaw_rate);
    control.sendSpeed(vec);
}

void Controller::positionControllLoop()
{
    //TODO add position mode.
}



void Controller::setMode(ControllerMode new_mode)
{
    for(auto pid: pids)
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
