#pragma once

#define USE_QUATERIONS 1

/// @brief Controller constants
namespace def {

/// @brief Step time of controller. Step of controller and EKF calculations
const double STEP_TIME = 0.003;

/// @brief How often send demands in response to stick command
const int INFO_PERIOD = 2;
}
