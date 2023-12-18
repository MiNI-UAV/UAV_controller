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
    //Position of uptown tower on the city map.
    const Eigen::Vector3d target{1530.0,-330.0,-300.0};

    static constexpr double detection_limit = std::numbers::pi/3.0;
};