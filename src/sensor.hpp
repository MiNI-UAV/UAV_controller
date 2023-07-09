#pragma once
#include <zmq.hpp>
#include <thread>
#include <Eigen/Dense>
#include <mutex>
#include <random>
#include "NS.hpp"

class NS;

class Sensor
{
    public:
        Sensor(zmq::context_t* ctx, std::string uav_address, NS& navisys, bool& run, double sd1, double sd2, std::string name, std::string topic);
        ~Sensor();

        zmq::context_t* _ctx;
        std::string _uav_address;
        NS& _navisys;
        bool& _run;
        std::normal_distribution<double> _dist1;
        std::normal_distribution<double> _dist2;
        std::thread listener;
        const std::string _name;
        const std::string _topic;

        double error1();
        double error2();
        void setPosition(Eigen::Vector3d newValue);
        void setOrientation(Eigen::Vector3d newValue);
        void setLinearVelocity(Eigen::Vector3d newValue);
        void setAngularVelocity(Eigen::Vector3d newValue);
        void setLinearAcceleration(Eigen::Vector3d newValue);
        void setAngularAcceleraton(Eigen::Vector3d newValue);


        virtual void handleMsg(std::string msg) = 0;
        void listenerJob();
};

class GNSS : public Sensor
{
    public:
        GNSS(zmq::context_t* ctx, std::string uav_address, NS& navisys, bool& run, double sd);
    private:
        void handleMsg(std::string msg) override;
};

class Gyroscope : public Sensor
{
    public:
        Gyroscope(zmq::context_t* ctx, std::string uav_address, NS& navisys, bool& run, double sd);
    private:
        void handleMsg(std::string msg) override;
};

class Accelerometer : public Sensor
{
    public:
        Accelerometer(zmq::context_t* ctx, std::string uav_address, NS& navisys, bool& run, double sd1, double sd2);
    private:
        void handleMsg(std::string msg) override;
};

class MagicOrientationSensor : public Sensor
{
    public:
        MagicOrientationSensor(zmq::context_t* ctx, std::string uav_address, NS& navisys, bool& run);
    private:
        void handleMsg(std::string msg) override;
};

class MagicLinearVelocitySensor : public Sensor
{
    public:
        MagicLinearVelocitySensor(zmq::context_t* ctx, std::string uav_address, NS& navisys, bool& run);
    private:
        void handleMsg(std::string msg) override;
};
