#pragma once
#include <limits>

enum AntiWindUpMode
{
    None,Clamping
};

//Inspiration : https://gist.github.com/bradley219/5373998
class PID
{
    public:
        PID(double dt, double Kp, double Ki, double Kd,
         double min = std::numeric_limits<double>::min(),
         double max = std::numeric_limits<double>::max(),
         AntiWindUpMode antiWindUp = AntiWindUpMode::Clamping);
        ~PID();
        double calc(double error);
        void clear();


    private:
        double _dt;
        double _max;
        double _min;
        double _Kp;
        double _Kd;
        double _Ki;
        double _pre_error;
        double _integral;
        AntiWindUpMode _antiWindUp;
};