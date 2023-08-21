#pragma once
#include <Eigen/Dense>
#include "rapidxml/rapidxml.hpp"
#include <map>
#include "common.hpp"

#define USE_QUATERIONS 1

const int step_time = 3;

struct SensorParams
{
    std::string name;
    double sd;
    Eigen::Vector3d bias;
    double refreshTime;
};

struct AHRSParams
{
    std::string type;
    double alpha;
    double Q;
    double R;
};

struct EKFScalers
{
    double predictScaler;
    double updateScaler;
    double baroScaler;
    double zScaler;
};

struct Params
{
    public:
        Params();
        ~Params() = default;
        void loadConfig(std::string configFile);

        std::string name;
        int noOfRotors;
        double hoverRotorSpeed;
        double maxRotorSpeed;
        std::map<std::string,PID> pids;
        std::function<Eigen::VectorXd(double,double,double,double)> mixer;

        std::vector<SensorParams> sensors;
        AHRSParams ahrs;
        EKFScalers ekf;

    private:      
        void parseSensors(rapidxml::xml_node<>* sensorNode);
        void parseAHRS(rapidxml::xml_node<>* AHRSNode);
        void parseEKF(rapidxml::xml_node<>* EKFNode);
};