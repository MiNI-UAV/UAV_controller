#include <zmq.hpp>

class Control
{
    public:
        Control(zmq::context_t* ctx, std::string uav_address);
        ~Control();

    private:
        zmq::socket_t sock;
};