#include <iostream>
#include "GPS.hpp"
#include "gyro.hpp"
#include "mixers.hpp"
#include "control.hpp"
#include <Eigen/Dense>

int main()
{
	zmq::context_t ctx;
	std::string uav_address = "ipc:///tmp/default";
	GPS_AH gps(&ctx,uav_address);
	Gyro gyro(&ctx,uav_address);
	Control control(&ctx,uav_address);


	while(1)
	{
		std::cout << "Pos:\n"<< gps.getGPSPos().transpose() << std::endl << std::endl;
		std::cout << "Vel:\n"<< gps.getGPSVel().transpose() << std::endl << std::endl;
		std::cout << "Angular:\n"<< gyro.getAngularVel().transpose() << std::endl << std::endl;
		Eigen::VectorXd vec;
		vec.setConstant(4, 2.1234);
		control.sendSpeed(vec);
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
}
