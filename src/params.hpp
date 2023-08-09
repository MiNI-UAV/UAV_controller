#pragma once
#include <Eigen/Dense>
#include "rapidxml/rapidxml.hpp"
#include <map>
#include "UAV_common/PID.hpp"

const int step_time = 3;

struct Params
{
    public:
        Params();
        ~Params();
        void loadConfig(std::string configFile);

        std::string name;
        int noOfRotors;
        double hoverRotorSpeed;
        double maxRotorSpeed;
        std::map<std::string,PID> pids;
        std::function<Eigen::VectorXd(double,double,double,double)> mixer;
        

    private:      
        void setMass(rapidxml::xml_node<> * interiaNode);
};