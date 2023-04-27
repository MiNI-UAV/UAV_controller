#include "PID.hpp"
#include <limits>
#include <algorithm>

PID::PID(double dt, double Kp, double Ki, double Kd,
         double min = std::numeric_limits<double>::min(),
         double max = std::numeric_limits<double>::max()):
    _dt(dt),
    _max(max),
    _min(min),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _pre_error(0),
    _integral(0)
{     
}

PID::~PID()
{
}

double PID::calc(double error)
{
    double Pout = _Kp * error;

    //missing Anti-windup
    _integral += error * _dt;
    double Iout = _Ki * _integral;

    double derivative = (error - _pre_error) / _dt;
    double Dout = _Kd * derivative;

    double output = Pout + Iout + Dout;

    output = std::clamp(output,_min,_max);

    _pre_error = error;

    return output;
}
