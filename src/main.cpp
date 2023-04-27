#include <iostream>
#include "GPS.hpp"
#include <Eigen/Dense>

int main()
{
	zmq::context_t ctx;
	std::string uav_address = "ipc:///tmp/default/state";
	GPS_AH gps(&ctx,uav_address);


	while(1)
	{
		std::cout << "Pos:\n"<< gps.getGPSPos().transpose() << std::endl << std::endl;
		std::cout << "Vel:\n"<< gps.getGPSVel().transpose() << std::endl << std::endl;
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
}
