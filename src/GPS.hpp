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
        Eigen::Vector3d getGPS();
        Eigen::Vector3d getAH();
        void handleMsg(std::string msg);

    private:
        std::thread listener;
        Eigen::Vector3d pos;
        Eigen::Vector3d orientation;
        std::mutex mtx;

        void setPosAndOrient(Eigen::Vector3d newPos, Eigen::Vector3d newOrientation);
        Eigen::Vector3d GPSError(Eigen::Vector3d pos);
        Eigen::Vector3d AHError(Eigen::Vector3d orientation);

};