#define LOGGER_MASK 5

#include <iostream>
#include <fstream>
#include <cxxopts.hpp>
#include <thread>
#include <chrono>
#include <filesystem>
#include "zmq.hpp"
#include "controller/controller.hpp"
#include "common.hpp"
#include "params.hpp"

std::string log_path = "logs/";

/// @brief Parse CL arguments
/// @param argc number of argument
/// @param argv argument array
/// @param params pointer to UAVparams instant that should be filled
/// @param p internal params reference
void parseArgs(int argc, char** argv, UAVparams* params, Params& p)
{
    cxxopts::Options options("controller", "Process representing control system of one UAV");
    options.add_options()
		("c,config", "Path of config file", cxxopts::value<std::string>()->default_value("config.xml"))
        ("n,name", "Override name from config", cxxopts::value<std::string>()->default_value(""))
        ("dt", "Step time of simulation in ms. Default: 1 ms", cxxopts::value<int>())
        ("h,help", "Print usage");
    auto result = options.parse(argc, argv);
    if(result.count("help"))
    {
        std::cout << options.help() << std::endl;
        exit(0);
    }
    if(result.count("dt"))
    {
        p.STEP_TIME = result["dt"].as<int>()/1000.0;
        std::cout << "Step time changed to " << p.STEP_TIME << "s" << std::endl;
    }
    params->loadConfig(result["config"].as<std::string>().c_str());
    if(result.count("name"))
    {
        params->name = result["name"].as<std::string>();
    }
    std::cout << "Name: " << params->name <<std::endl;
}

int main(int argc, char** argv)
{
	zmq::context_t ctx;
    UAVparams params;
    Params p{};
    parseArgs(argc,argv,&params, p);
    Logger::setLogDirectory(params.name);
	std::string uav_address = "ipc:///tmp/" + std::string(params.name);
    std::string folder = "/tmp/" + std::string(params.name);
    std::cout << "Looking for folder: " << folder << std::endl;
    while(!std::filesystem::exists(folder)) std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::cout << "Comunication folder found!" << std::endl;
	ControlSystem controller(&ctx,uav_address);
	controller.run();
}
