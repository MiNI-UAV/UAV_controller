#include <iostream>
#include <cxxopts.hpp>
#include "zmq.hpp"
#include "controller.hpp"
#include "params.hpp"

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
 //   if(result.count("config"))
 //   {
        params.loadConfig(result["config"].as<std::string>().c_str());
//    }
    if(result.count("name"))
    {
        std::string newName = result["name"].as<std::string>();
        params.setName(newName.c_str(),newName.length());
    }
    std::cout << "Name: " << params.name <<std::endl;
    return params;
}

int main(int argc, char** argv)
{
	zmq::context_t ctx;
    Params params;
    parseArgs(argc,argv,params);
	std::string uav_address = "ipc:///tmp/" + std::string(params.name);
	Controller controller(&ctx,uav_address,params);
	controller.run();
}
