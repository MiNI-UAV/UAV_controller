#include <iostream>
#include "zmq.hpp"
#include "controller.hpp"


int main()
{
	zmq::context_t ctx;
	std::string uav_address = "ipc:///tmp/drone1";
	Controller controller(&ctx,uav_address);
	controller.run();
}
