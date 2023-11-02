#pragma once
#include <Eigen/Dense>


/// @brief Calculates rotor demanded speed as result of multiplication mixer matrix and rates. Average speed is proportional to climb rate
/// @param climb_rate 
/// @param roll_rate 
/// @param pitch_rate 
/// @param yaw_rate 
/// @return Rotors demanded speed
Eigen::VectorXd applyMixerRotors(double  climb_rate, double roll_rate , double pitch_rate, double yaw_rate);

/// @brief Calculates rotor demanded speed as result of multiplication mixer matrix and rates. Average speed is proportional to throttle.
/// It's scaled to achieve hover at centered throttle
/// @param throttle 
/// @param roll_rate 
/// @param pitch_rate 
/// @param yaw_rate 
/// @return Rotors demanded speed
Eigen::VectorXd applyMixerRotorsHover(double  throttle, double roll_rate , double pitch_rate, double yaw_rate);



/// @brief Calculated demanded surfaces deflection result of multiplication mixer matrix and rates
/// @param throttle 
/// @param roll_rate 
/// @param pitch_rate 
/// @param yaw_rate 
/// @return demanded surfaces deflection
Eigen::VectorXd applyMixerSurfaces(double  throttle, double roll_rate , double pitch_rate, double yaw_rate);
