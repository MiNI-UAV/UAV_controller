#define LOGGER_MASK 5

#include <iostream>
#include <fstream>
#include <cxxopts.hpp>
#include <thread>
#include <chrono>
#include <filesystem>
#include "zmq.hpp"
#include "controller.hpp"
#include "params.hpp"

std::string log_path = "logs/";

Params parseArgs(int argc, char** argv, Params& params)
{

    cxxopts::Options options("controller", "Process representing PID controller of one UAV");
    options.add_options()
		("c,config", "Path of config file", cxxopts::value<std::string>()->default_value("config.xml"))
        ("n,name", "Override name from config", cxxopts::value<std::string>()->default_value(""))
        ("h,help", "Print usage");
    auto result = options.parse(argc, argv);
    if(result.count("help"))
    {
        std::cout << options.help() << std::endl;
        exit(0);
    }
    //if(result.count("config"))
    {
        params.loadConfig(result["config"].as<std::string>().c_str());
    }
    if(result.count("name"))
    {
        params.name = result["name"].as<std::string>();
    }
    std::cout << "Name: " << params.name <<std::endl;
    return params;
}

int main(int argc, char** argv)
{
	zmq::context_t ctx;
    Params params;
    parseArgs(argc,argv,params);
    Logger::setLogDirectory(params.name);
	std::string uav_address = "ipc:///tmp/" + std::string(params.name);
    std::string folder = "/tmp/" + std::string(params.name);
    std::cout << "Looking for folder: " << folder << std::endl;
    while(!std::filesystem::exists(folder)) std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::cout << "Comunication folder found!" << std::endl;
	Controller controller(&ctx,uav_address,params);
	controller.run();
}
