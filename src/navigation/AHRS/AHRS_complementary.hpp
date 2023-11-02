#pragma once
#include <Eigen/Dense>
#include <random>
#include "../environment.hpp"
#include "../sensors.hpp"
#include "common.hpp"
#include "../AHRS.hpp"

/// @brief Implementation of AHRS based on Complementary Filter
class AHRS_complementary : public AHRS
{
public:
    AHRS_complementary(Environment& env, double alpha);
    ~AHRS_complementary();

    Eigen::Matrix3d rot_bw() override;
    void update(Eigen::Vector3d gyro, Eigen::Vector3d acc, Eigen::Vector3d mag) override;

protected:
    const double alpha;
};
