#include <iostream>
#include "GPS.hpp"

int main()
{
	zmq::context_t ctx;
	std::string uav_address = "ipc:///tmp/default/state";
	GPS_AH gps(&ctx,uav_address);


	while(1)
	{
		std::cout << gps.getGPS() << "\n" << gps.getAH() << std::endl;
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
}
