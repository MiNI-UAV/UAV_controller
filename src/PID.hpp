
//Inspiration : https://gist.github.com/bradley219/5373998
class PID
{
    public:
        PID(double dt, double Kp, double Ki, double Kd, double min, double max);
        ~PID();
        double calc(double error);


    private:
        double _dt;
        double _max;
        double _min;
        double _Kp;
        double _Kd;
        double _Ki;
        double _pre_error;
        double _integral;
};