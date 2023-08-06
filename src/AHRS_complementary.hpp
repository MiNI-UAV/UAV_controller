#pragma once
#include <Eigen/Dense>
#include <random>
#include "environment.hpp"
#include "sensors.hpp"
#include "logger.hpp"
#include "AHRS.hpp"


class AHRS_complementary : public AHRS
{
public:
    AHRS_complementary(Environment& env, double alpha);
    ~AHRS_complementary();

    Eigen::Matrix3d rot_bw() override;
    void update() override;

protected:
    const double alpha;
};
