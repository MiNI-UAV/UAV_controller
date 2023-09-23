#pragma once
#include <Eigen/Dense>

Eigen::VectorXd applyMixerRotors(double  climb_rate, double roll_rate , double pitch_rate, double yaw_rate);
Eigen::VectorXd applyMixerRotorsHover(double  throttle, double roll_rate , double pitch_rate, double yaw_rate);
Eigen::VectorXd applyMixerSurfaces(double  throttle, double roll_rate , double pitch_rate, double yaw_rate);
