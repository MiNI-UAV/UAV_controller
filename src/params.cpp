#include <Eigen/Dense>
#include "params.hpp"
#include <iostream>
#include <fstream>
#include <filesystem>
#include <sstream>
#include <vector>
#include "rapidxml/rapidxml.hpp"
#include "mixers.hpp"

Params::Params() 
{
    name = "default";
    noOfRotors = 4;
}

Eigen::MatrixX4d  stringToMatrix(const std::string& input) {
    std::vector<double> values;
    std::istringstream iss(input);
    std::string token;

    while (std::getline(iss, token, ',')) {
        try {
            double value = std::stod(token);
            values.push_back(value);
        } catch (const std::exception& e) {
            std::cerr << "Invalid input: " << token << std::endl;
            return Eigen::MatrixXd();
        }
    }
    Eigen::Map<Eigen::MatrixX4d> matrix(values.data(), values.size()/4, 4);
    return matrix;
}

PID parsePID(rapidxml::xml_node<>* PIDNode)
{
    double P = 0,I = 0,D = 0,
    min = std::numeric_limits<double>::min(),
    max = std::numeric_limits<double>::max();

    for (rapidxml::xml_node<>* node = PIDNode->first_node(); node; node = node->next_sibling()) 
    {
        if(std::strcmp(node->name(),"P") == 0) P = std::stod(node->value());
        if(std::strcmp(node->name(),"I") == 0) I = std::stod(node->value());
        if(std::strcmp(node->name(),"D") == 0) D = std::stod(node->value());
        if(std::strcmp(node->name(),"min") == 0) min = std::stod(node->value());
        if(std::strcmp(node->name(),"max") == 0) max = std::stod(node->value());
    }
    return PID((double)step_time / 1000.0,P,I,D,min,max);
}

void Params::loadConfig(std::string configFile)
{
    if(!std::filesystem::exists(configFile))
    {  
        throw std::runtime_error("Config file not exist!");
    }
    std::cout << "Loading config" << std::endl;
    std::ifstream file(configFile);
    std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    rapidxml::xml_document<> doc;
    doc.parse<0>(&content[0]);
    rapidxml::xml_node<>* root = doc.first_node("params");
    for (rapidxml::xml_node<>* node = root->first_node(); node; node = node->next_sibling()) 
    {
        if(std::strcmp(node->name(),"name") == 0)
        {
            name.assign(node->value(), node->value_size());
        }
        if(std::strcmp(node->name(),"rotors") == 0)
        {
            for (rapidxml::xml_node<>* rotorNode = node->first_node(); rotorNode; rotorNode = rotorNode->next_sibling()) 
            {
                if(std::strcmp(rotorNode->name(),"no") == 0)
                {
                    noOfRotors = std::stod(rotorNode->value());
                }
            }
        }
        if(std::strcmp(node->name(),"PID") == 0)
        {
            for (rapidxml::xml_node<>* PIDNode = node->first_node(); PIDNode; PIDNode = PIDNode->next_sibling()) 
            {
                pids.insert(std::make_pair(PIDNode->name(),parsePID(PIDNode)));
            }
        }
        if(std::strcmp(node->name(),"control") == 0)
        {
            for (rapidxml::xml_node<>* controlNode = node->first_node(); controlNode; controlNode = controlNode->next_sibling()) 
            {
                if(std::strcmp(controlNode->name(),"maxSpeed") == 0)
                {
                    maxRotorSpeed = std::stod(controlNode->value());
                }
                if(std::strcmp(controlNode->name(),"hoverSpeed") == 0)
                {
                    hoverRotorSpeed = std::stod(controlNode->value());
                }
            }
        }
        if(std::strcmp(node->name(),"mixer") == 0)
        {
            Eigen::MatrixX4d mixerMatrix = stringToMatrix(node->value());
            double maxSpeed = maxRotorSpeed;
            mixer = [mixerMatrix, maxSpeed](double c, double r, double p, double y) {return controlMixer(mixerMatrix,c,r,p,y,maxSpeed);};
        }
        if(std::strcmp(node->name(),"navi") == 0)
        {
            for (rapidxml::xml_node<>* naviNode = node->first_node(); naviNode; naviNode = naviNode->next_sibling()) 
            {
                if(std::strcmp(naviNode->name(),"sensors") == 0)
                {
                    parseSensors(naviNode);
                }
                if(std::strcmp(naviNode->name(),"AHRS") == 0)
                {
                    parseAHRS(naviNode);
                }
                if(std::strcmp(naviNode->name(),"EKF") == 0)
                {
                    parseEKF(naviNode);
                }
            }
        }  
    }
    std::cout << "Loading config done!" << std::endl;
}

void Params::parseSensors(rapidxml::xml_node<> *sensorNode)
{
    for (rapidxml::xml_node<>* node = sensorNode->first_node(); node; node = node->next_sibling()) 
    {
        SensorParams sensor;
        sensor.name = std::string(node->name());
        for (rapidxml::xml_node<>* elem = node->first_node(); elem; elem = elem->next_sibling()) 
        {
            if(std::strcmp(elem->name(),"sd") == 0) sensor.sd = std::stod(elem->value());
            if(std::strcmp(elem->name(),"refreshTime") == 0) sensor.refreshTime = std::stod(elem->value());
            if(std::strcmp(elem->name(),"bias") == 0)
            {
                double x,y,z;
                std::sscanf(elem->value(),"%lf %lf %lf",&x,&y,&z);
                sensor.bias << x,y,z;
            } 
        }
        sensors.push_back(std::move(sensor));
    }
}

void Params::parseAHRS(rapidxml::xml_node<> *AHRSNode)
{
    for (rapidxml::xml_node<>* node = AHRSNode->first_node(); node; node = node->next_sibling()) 
    {
        if(std::strcmp(node->name(),"type") == 0) ahrs.type = std::string(node->value());
        if(std::strcmp(node->name(),"alpha") == 0) ahrs.alpha = std::stod(node->value());
        if(std::strcmp(node->name(),"Q") == 0) ahrs.Q = std::stod(node->value());
        if(std::strcmp(node->name(),"R") == 0) ahrs.R = std::stod(node->value());
    }
}

void Params::parseEKF(rapidxml::xml_node<> *EKFNode)
{
    for (rapidxml::xml_node<>* node = EKFNode->first_node(); node; node = node->next_sibling()) 
    {
        if(std::strcmp(node->name(),"predictScaler") == 0) ekf.predictScaler = std::stod(node->value());
        if(std::strcmp(node->name(),"updateScaler") == 0) ekf.updateScaler = std::stod(node->value());
        if(std::strcmp(node->name(),"baroScaler") == 0) ekf.baroScaler = std::stod(node->value());
        if(std::strcmp(node->name(),"zScaler") == 0) ekf.zScaler = std::stod(node->value());
    }
}
