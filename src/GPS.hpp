#pragma once
#include <zmq.hpp>
#include <thread>
#include <Eigen/Dense>
#include <mutex>


class GPS_AH
{
    public:
        GPS_AH(zmq::context_t* ctx, std::string uav_address);
        ~GPS_AH();
        Eigen::Vector3d getGPSPos();
        Eigen::Vector3d getGPSVel();
        Eigen::Vector3d getAH();
        void handlePosMsg(std::string msg);
        void handleVelMsg(std::string msg);

    private:
        std::thread posListener;
        std::thread vnListener;
        Eigen::Vector3d pos;
        Eigen::Vector3d vel;
        Eigen::Vector3d orientation;
        std::mutex mtxPos, mtxVel, mtxOri;

        void setPosAndOrient(Eigen::Vector3d newPos, Eigen::Vector3d newOrientation);
        void setVel(Eigen::Vector3d newVel);
        Eigen::Vector3d GPSPosError(Eigen::Vector3d pos);
        Eigen::Vector3d GPSVelError(Eigen::Vector3d vel);
        Eigen::Vector3d AHError(Eigen::Vector3d orientation);


};