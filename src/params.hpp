#pragma once
#include <Eigen/Dense>
#include "rapidxml/rapidxml.hpp"
#include <map>
#include "PID.hpp"

struct Params
{
    public:
        Params();
        ~Params();
        void loadConfig(std::string configFile);
        void setName(const char* newName, size_t sz);

        char* name;
        int noOfRotors;
        double hoverRotorSpeed;
        double maxRotorSpeed;
        std::map<std::string,PID> pids;
        std::function<Eigen::VectorXd(double,double,double,double)> mixer;
        

    private:      
        void setMass(rapidxml::xml_node<> * interiaNode);
};