#include <zmq.hpp>
#include <Eigen/Dense>

class Control
{
    public:
        Control(zmq::context_t* ctx, std::string uav_address);
        void prepare();
        void start();
        void stop();
        void recv();
        void sendSpeed(Eigen::VectorXd speeds);
        void sendSurface(Eigen::VectorXd angels);
        void startJet(int index);
        void sendHinge(char type, int index, int hinge_index, double value);
        ~Control();

    private:
        void sendVectorXd(std::string prefix, Eigen::VectorXd vec);
        void sendString(std::string msg);

        zmq::socket_t sock;
};