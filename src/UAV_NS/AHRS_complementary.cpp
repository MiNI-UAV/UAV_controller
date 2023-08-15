#include "AHRS_complementary.hpp"
#include <Eigen/Dense>
#include <random>
#include <iostream>
#include "environment.hpp"
#include "sensors.hpp"
#include "common.hpp"

AHRS_complementary::AHRS_complementary(Environment& env, double alpha):
    AHRS(env),
    alpha{alpha}
{
    logger.setFmt("Time, Roll, Pitch, Yaw");
}

AHRS_complementary::~AHRS_complementary() 
{
}



Eigen::Matrix3d calcRnb(Eigen::Vector3d ori)
{
    double cf = cos(ori(0));
    double sf = sin(ori(0));
    double ct = cos(ori(1));
    double st = sin(ori(1));
    double cp = cos(ori(2));
    double sp = sin(ori(2));
    Eigen::Matrix3d R_nb;
    R_nb << ct*cp           , ct*sp           , -st,
            sf*st*cp - cf*sp, sf*st*sp + cf*cp, sf*ct,
            cf*st*cp + sf*sp, cf*st*sp - sf*cp, cf*ct;
    return R_nb;
}

Eigen::Matrix3d calcRbn(Eigen::Vector3d ori)
{
    double cf = cos(ori(0));
    double sf = sin(ori(0));
    double ct = cos(ori(1));
    double st = sin(ori(1));
    double cp = cos(ori(2));
    double sp = sin(ori(2));
    Eigen::Matrix3d R_bn;
    R_bn << ct*cp, sf*st*cp - cf*sp, cf*st*cp + sf*sp,
            ct*sp, sf*st*sp + cf*cp, cf*st*sp - sf*cp,
            -st  , sf*ct           , cf*ct;
    return R_bn;
}

Eigen::Matrix3d AHRS_complementary::rot_bw()
{
    std::scoped_lock lck(mtxOri);
    return calcRbn(ori_est);
}

Eigen::Matrix3d calcTom(Eigen::Vector3d ori)
{
    double cf = cos(ori(0));
    double sf = sin(ori(0));
    double ct = cos(ori(1));
    double tt = tan(ori(1));
    Eigen::Matrix3d Tom;
    Tom << 1, sf*tt, cf*tt,
           0, cf           , -sf,
           0, sf/ct, cf/ct;
    return Tom;
}

void clampOrientation(Eigen::Vector3d& vec)
{
    for (size_t i = 0; i < 3; i++)
    {
        double x = fmod(vec(i) + std::numbers::pi,2*std::numbers::pi);
        if (x < 0)
            x += 2*std::numbers::pi;
        vec(i) =  x - std::numbers::pi;
    }
}

void AHRS_complementary::update(Eigen::Vector3d gyro, Eigen::Vector3d acc, Eigen::Vector3d mag)
{
    static double last_time = 0.0;
    static Eigen::Vector3d ori_gyro = Eigen::Vector3d(0.0,0.0,0.0);
    double time = env.getTime();
    if(time == 0.0) return;

    ori_gyro += (time-last_time)*(calcTom(ori_gyro)*gyro);
    last_time = time;
    clampOrientation(ori_gyro);
    Eigen::Vector3d ori_acc = Eigen::Vector3d(
        atan2(acc.y(), acc.z()),
        atan2(-acc.x(), acc.y()*sin(ori_gyro.x()) + acc.z()*cos(ori_gyro.x())),
        atan2(mag.z()*sin(ori_gyro.x()) - mag.y()*cos(ori_gyro.x()),
        mag.x()*cos(ori_gyro.y()) 
        + mag.y()*sin(ori_gyro.y())*sin(ori_gyro.x())
        + mag.z()*sin(ori_gyro.y())*cos(ori_gyro.x())
        )
    );
    Eigen::Vector3d new_ori;
    for(int i = 0; i < 3; i++)
    {
        if(std::abs(ori_gyro(i)-ori_acc(i)) > std::numbers::pi)
        {
            if(ori_acc(i) > ori_gyro(i))
            {
                ori_gyro(i) += 2*std::numbers::pi;
            }
            else
            {
                ori_acc(i) += 2*std::numbers::pi;
            }
            new_ori(i) = alpha * ori_gyro(i) + (1.0-alpha) * ori_acc(i);
            
        }           
        else{
            new_ori(i) = alpha * ori_gyro(i) + (1.0-alpha) * ori_acc(i);
        }
    }
    clampOrientation(new_ori);
    mtxOri.lock();
    ori_est = new_ori;
    mtxOri.unlock();
    logger.log(time,{new_ori, ori_gyro, ori_acc});
}
