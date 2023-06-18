#include <Eigen/Dense>
#include "params.hpp"
#include <iostream>
#include <fstream>
#include <filesystem>
#include <sstream>
#include <vector>
#include "rapidxml/rapidxml.hpp"
#include "mixers.hpp"
#include "PID.hpp"

/// @brief Initialize default data
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

PID Params::parsePID(rapidxml::xml_node<>* PIDNode)
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
    return PID(step_time/1000.0,P,I,D,min,max);
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
            std::cout << mixerMatrix << std::endl;
            mixer = [mixerMatrix,this](double c, double r, double p, double y) {return controlMixer(mixerMatrix,c,r,p,y,maxRotorSpeed);};
        }
        
    }
}

Params::~Params()
{
}