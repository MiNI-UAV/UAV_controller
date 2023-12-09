#pragma once
#include "../controller_loop.hpp"

class ControllerLoopRAUTOLAUNCH: public ControllerLoop
{
public:
    ControllerLoopRAUTOLAUNCH();

    void job(
        [[maybe_unused]] std::map<std::string,std::unique_ptr<Controller>>& controllers,
        Control& control,
        [[maybe_unused]] NS& navisys) override;

protected:
};