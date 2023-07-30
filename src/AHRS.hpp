#include <Eigen/Dense>
#include <random>
#include "environment.hpp"
#include "sensors.hpp"
#include "logger.hpp"

class AHRS
{
public:
    AHRS(Environment& env);
    ~AHRS();

private:
    double alfa = 0.98;

    Eigen::Vector3d ori_est;
    std::mutex mtxOri;

    Environment& env;
    Logger logger;

    Accelerometer acc;
    Gyroscope gyro;
    Magnetometer mag;

    std::atomic_bool run;
    void listenerJob();
    std::thread listener;
};
