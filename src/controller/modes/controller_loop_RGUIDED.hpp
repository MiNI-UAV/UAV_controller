#pragma once
#include "../controller_loop.hpp"

class ControllerLoopRGUIDED: public ControllerLoop
{
public:
    ControllerLoopRGUIDED();

    void job(
        [[maybe_unused]] std::map<std::string,std::unique_ptr<Controller>>& controllers,
        Control& control,
        [[maybe_unused]] NS& navisys) override;

    std::string demandInfo() override;

protected:
    const Eigen::Vector3d target;
    
    static constexpr double detection_limit = std::numbers::pi/3.0;
};