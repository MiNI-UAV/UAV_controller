#include <iostream>
#include <Eigen/Dense>
#include "GPS.hpp"
#include "gyro.hpp"
#include "mixers.hpp"
#include "control.hpp"
#include "PID.hpp"
#include "timed_loop.hpp"
#include "state.hpp"

int main()
{
	zmq::context_t ctx;
	std::string uav_address = "ipc:///tmp/drone1";
	const int controlServerPort = 10001;
	constexpr int step_time = 3; //ms

	GPS_AH gps(&ctx,uav_address);
	Gyro gyro(&ctx,uav_address);
	Control control(&ctx,uav_address);
	State state(&ctx,controlServerPort);
	Status status = Status::running;



	PID pidZ(step_time/1000.0, 2.122, 0.035, -0.387, -1000, 1000);
	PID pidFi(step_time/1000.0, 9.584, 0.798, 0.192, -1000, 1000);
	PID pidTheta(step_time/1000.0, 5.191, 0.228, 0.127, -1000, 1000);
	PID pidPsi(step_time/1000.0, 5.288, 0.230, -0.151, -1000, 1000);

	PID pidW(step_time/1000.0, -3556.149, -538.572, -112.917, 0, 1000);
	PID pidRoll(step_time/1000.0, -6.249, -0.904, -0.219, -250, 250);
	PID pidPitch(step_time/1000.0, 6.304, 1.174, 0.433, -250, 250);
	PID pidYaw(step_time/1000.0, 112.662, 22.778, 3.419, -250, 250);

	TimedLoop loop(std::round(step_time), [&](){
		Eigen::Vector3d pos = gps.getGPSPos();
		Eigen::Vector3d vel = gps.getGPSVel();
		Eigen::Vector3d ori = gps.getAH();
		Eigen::Vector3d angVel = gyro.getAngularVel();

		double demandedVz = pidZ.calc(state.demandedZ - pos(2));
		double demandedP = pidFi.calc(state.demandedFi - ori(0));
		double demandedQ = pidTheta.calc(state.demandedTheta - ori(1));
		double demandedR = pidPsi.calc(state.demandedPsi - ori(2));

        double climb_rate = pidW.calc(demandedVz-vel(2));
		double roll_rate = pidRoll.calc(demandedP-angVel(0));
		double pitch_rate = pidPitch.calc(demandedQ-angVel(1));
		double yaw_rate = pidYaw.calc(demandedR-angVel(2));
		Eigen::VectorXd vec = controlMixer4(climb_rate,roll_rate,pitch_rate,yaw_rate);
		control.sendSpeed(vec);
    }, status);

	loop.go();
}
