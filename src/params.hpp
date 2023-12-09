#pragma once
#include <string>

/// @brief Simulation parameters
class Params
{
public:
    /// @brief Constructor
    Params();

    Params(const Params&) = delete; // no copies
    Params& operator=(const Params&) = delete; // no self-assignments
    Params(Params&&) = delete; // no moves

    /// @brief Deconstructor
    ~Params();

    /// @brief Step time of simulation. Step of ODE solving methods
    double STEP_TIME;

    /// @brief Get singleton of Params.
    /// @return const pointer to Params instance. Return nullptr if not initialized
    static const Params* getSingleton();

private:
    static Params* _singleton;
};
