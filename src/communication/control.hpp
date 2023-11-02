#pragma once

#include <zmq.hpp>
#include <Eigen/Dense>
#include <atomic>
#include <thread>
#include <functional>
#include "../controller/controller.hpp"

class Controller;

/// @brief Control command listener & sender
class Control
{
    public:
        /// @brief Constructor
        /// @param ctx zero mq context
        /// @param uav_address address to REP socket in simulation of controller uav 
        /// @param controller pointer to controller instance
        Control(zmq::context_t *ctx, std::string uav_address, Controller* controller);

        /// @brief Deconstructor
        ~Control();

        /// @brief Sends ping command
        void prepare();

        /// @brief Sends start command
        void start();

        /// @brief Sends stop command
        void stop();

        /// @brief Recivers reply and check if it contains "ok" phrase
        void recv();

        /// @brief Sends new demanded rotors speed
        /// @param speeds vector of demanded speeds
        void sendSpeed(Eigen::VectorXd speeds);

        /// @brief Sends new demanded surface deflactions
        /// @param speeds vector of surface deflactions
        void sendSurface(Eigen::VectorXd angels);


        /// @brief Sends command to start jet engine of given index
        /// @param index jet engine index
        void startJet(int index);


        /// @brief Sends command to control hinge deflaction
        /// @param type hinge type: 'r' - rotor, 'j' - jet
        /// @param index drive index
        /// @param hinge_index hinge index
        /// @param value new deflection
        void sendHinge(char type, int index, int hinge_index, double value);

        /// @brief Handle incomming control message - message that instruct controller what to do
        /// @param msg message content
        /// @return reply to message
        std::string handleMsg(std::string msg);

    private:
        void sendVectorXd(std::string prefix, Eigen::VectorXd vec);
        void sendString(std::string msg);
        std::string handleControl(std::string content);
        std::string handleMode(std::string content);
        std::string handleJoystick(std::string content);

        bool run;
        std::thread orderServer;
        zmq::socket_t sock;
        Controller* _controller;
};