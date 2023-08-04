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
    AHRS_complementary(Environment& env, int updatePeriodInMs, double alpha);
    ~AHRS_complementary();

    void update() override;

private:
    const double alpha;
};
