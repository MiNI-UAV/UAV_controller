#pragma once
#include <Eigen/Dense>
#include "environment.hpp"
#include "sensors.hpp"
#include "logger.hpp"
#include "AHRS.hpp"


class AHRS_EKF : public AHRS
{
public:
    AHRS_EKF(Environment& env, int updatePeriodInMs);
    ~AHRS_EKF();

    Eigen::Matrix3d rot_bw() override;

protected:

    void update() override;

    // q0, q1, q2, q3, bx, by, bz
    Eigen::Vector<double,7> x;
    Eigen::Matrix<double,7,7> P;
    Eigen::Matrix<double,7,7> Q;
    Eigen::Matrix<double,6,6> R;

    Eigen::Vector4d q();
    Eigen::Vector3d quaterionToRPY(Eigen::Vector4d q);
};