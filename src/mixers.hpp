#pragma once
#include <Eigen/Dense>

Eigen::VectorXd controlMixer4(double climb_rate, double roll_rate , double pitch_rate, double yaw_rate, double maxSpeed = 1000.0);