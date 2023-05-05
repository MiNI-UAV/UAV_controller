#include <zmq.hpp>
#include <Eigen/Dense>

class Control
{
    public:
        Control(zmq::context_t* ctx, std::string uav_address);
        void prepare();
        void start();
        void stop();
        void sendSpeed(Eigen::VectorXd speeds);
        ~Control();

    private:
        zmq::socket_t sock;
};