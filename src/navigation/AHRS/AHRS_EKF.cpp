#include "AHRS_EKF.hpp"
#include <Eigen/Dense>
#include <random>
#include <iostream>

AHRS_EKF::AHRS_EKF(Environment &env, double Q_scaler, double R_scaler):
    AHRS(env)
{
    logger.setFmt("Time, Roll, Pitch, Yaw, q1, q2, q3, q4, bx, by, bz");
    x.setZero();
    x.head<4>() = RPYToQuaterion(ori_est);
    Q.setIdentity();
    Q *= Q_scaler;
    R.setIdentity();
    R *= R_scaler;
    P = Q;
}

AHRS_EKF::~AHRS_EKF()
{
}

Eigen::Vector3d AHRS_EKF::getGyroBias()
{
    return x.tail<3>();
}

Eigen::Matrix3d AHRS_EKF::rot_bw()
{
    mtxOri.lock();
    Eigen::Vector4d quat = q();
    mtxOri.unlock();
    Eigen::Matrix3d Rbw;
    Rbw << quat(0)*quat(0) + quat(1)*quat(1) - quat(2)*quat(2) - quat(3)*quat(3), 2*(quat(1)*quat(2) - quat(0)*quat(3))                     , 2*(quat(1)*quat(3) + quat(0)*quat(2)),
           2*(quat(1)*quat(2) + quat(0)*quat(3))                    , quat(0)*quat(0) - quat(1)*quat(1) + quat(2)*quat(2) - quat(3)*quat(3) , 2*(quat(2)*quat(3) - quat(0)*quat(1)),
           2*(quat(1)*quat(3) - quat(0)*quat(2))                    , 2*(quat(2)*quat(3) + quat(0)*quat(1))                     , quat(0)*quat(0) - quat(1)*quat(1) - quat(2)*quat(2) + quat(3)*quat(3);
    return Rbw;
}

Eigen::Vector4d AHRS_EKF::q()
{
    return x.head<4>();
}

Eigen::Matrix<double,4,3> S(Eigen::Vector4d q)
{
    Eigen::Matrix<double,4,3> s;
    s.setZero();
    s << -q(1), -q(2), -q(3),
          q(0), -q(3),  q(2),
          q(3),  q(0), -q(1),
         -q(2),  q(1),  q(0);
    return s;
}

Eigen::Matrix<double,6,7> C(Eigen::Vector4d q)
{
    Eigen::Matrix<double,6,7> c;
    c.setZero();
    c << -q(2),  q(3), -q(0),  q(1), 0.0, 0.0, 0.0,
          q(1),  q(0),  q(3),  q(2), 0.0, 0.0, 0.0,
          q(0), -q(1), -q(2),  q(3), 0.0, 0.0, 0.0,
         -q(0), -q(1),  q(2),  q(3), 0.0, 0.0, 0.0,
          q(3), -q(2), -q(1),  q(0), 0.0, 0.0, 0.0,
         -q(2), -q(3), -q(0), -q(1), 0.0, 0.0, 0.0; 
    return -2*c;
}



void AHRS_EKF::update(Eigen::Vector3d gyro, Eigen::Vector3d acc, Eigen::Vector3d mag) 
{
    static double last_update = 0.0;
    double time = env.getTime();
    if(time == 0.0) return;


    Eigen::Vector<double,6> y;
    y << acc.normalized(), mag.normalized();

    Eigen::Matrix<double,4,3> TS2 = ((time-last_update)/2.0)*S(q());
    Eigen::Matrix<double,7,7> A;
    A.setIdentity();
    A.block<4,3>(0,4) = -TS2;
    Eigen::Matrix<double,7,3> B;
    B.setZero();
    B.block<4,3>(0,0) = TS2;
    
    //Predict
    Eigen::Vector<double,7> xDash = A*x + B*gyro;
    Eigen::Matrix<double,7,7> PDash = A*P*A.transpose() + Q;

    //Update
    // Eigen::Matrix<double,6,7> C_val = C(xDash.head<4>());
    // Eigen::Matrix<double,6,6> inv_den = (C_val*PDash*C_val.transpose() + R).inverse();
    // Eigen::Matrix<double,7,6> K = (PDash*C_val.transpose())*inv_den;
    // x = xDash + K*(y-C_val*xDash);
    // Eigen::Matrix<double,7,7> I;
    // I.setIdentity();
    // P = (I - K*C_val)*PDash;
    x = xDash;
    P = PDash;

    Eigen::Vector3d ori = quaterionToRPY(q());
    mtxOri.lock();
    ori_est = ori;
    mtxOri.unlock();
    logger.log(time,{ori,x});
    last_update = time;
}

Eigen::Vector3d AHRS_EKF::quaterionToRPY(Eigen::Vector4d q)
{
    double roll  = atan2(2*(q(2)*q(3) + q(0)*q(1)), 1 - 2*(q(1)*q(1) + q(2)*q(2)));
    double pitch = asin(2*(q(0)*q(2)-q(1)*q(3)));
    double yaw   = atan2(2*(q(1)*q(2) + q(0)*q(3)), 1 - 2*(q(2)*q(2) + q(3)*q(3))); 
    return Eigen::Vector3d(roll,pitch,yaw);
}

Eigen::Vector4d AHRS_EKF::RPYToQuaterion(Eigen::Vector3d RPY)
{
    Eigen::Vector4d quat;
    double halfRoll = RPY(0) * 0.5;
    double halfPitch = RPY(1) * 0.5;
    double halfYaw = RPY(2) * 0.5;
    double sinHalfRoll = std::sin(halfRoll);
    double cosHalfRoll = std::cos(halfRoll);
    double sinHalfPitch = std::sin(halfPitch);
    double cosHalfPitch = std::cos(halfPitch);
    double sinHalfYaw = std::sin(halfYaw);
    double cosHalfYaw = std::cos(halfYaw);

    quat(0) = cosHalfRoll * cosHalfPitch * cosHalfYaw + sinHalfRoll * sinHalfPitch * sinHalfYaw; // w
    quat(1) = sinHalfRoll * cosHalfPitch * cosHalfYaw - cosHalfRoll * sinHalfPitch * sinHalfYaw; // x
    quat(2) = cosHalfRoll * sinHalfPitch * cosHalfYaw + sinHalfRoll * cosHalfPitch * sinHalfYaw; // y
    quat(3) = cosHalfRoll * cosHalfPitch * sinHalfYaw - sinHalfRoll * sinHalfPitch * cosHalfYaw; // z
    return quat;
}
