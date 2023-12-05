#pragma once
#include "../controller_loop.hpp"

class ControllerLoopRAUTOLAUNCH: public ControllerLoop
{
public:
    ControllerLoopRAUTOLAUNCH();

    void job(
        [[maybe_unused]] std::map<std::string,PID>& pids,
        Control& control,
        [[maybe_unused]] NS& navisys) override;

protected:
};