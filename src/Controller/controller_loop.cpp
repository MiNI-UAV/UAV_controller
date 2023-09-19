#include "controller_loop.hpp"

#include "controller_loop_NONE.hpp"
#include "controller_loop_QACRO.hpp"
#include "controller_loop_QANGLE.hpp"
#include "controller_loop_QPOS.hpp"

ControllerLoop::ControllerLoop(ControllerMode mode):
    _mode{mode}
{}

void ControllerLoop::job(
  [[maybe_unused]] State* state,
  [[maybe_unused]] std::map<std::string,PID>& pids,
  [[maybe_unused]] Control& control,
  [[maybe_unused]] NS& navisys
) 
{
    Eigen::VectorXd vec = applyMixerRotors(0.0,0.0,0.0,0.0);
    control.sendSpeed(vec);
}

ControllerLoop *ControllerLoop::ControllerLoopFactory(ControllerMode mode) {
    switch (mode)
    {
    case ControllerMode::NONE:
      return new ControllerLoopNONE();
    case ControllerMode::QPOS:
      return new ControllerLoopQPOS();
    case ControllerMode::QANGLE:
      return new ControllerLoopQANGLE();
    case ControllerMode::QACRO:
      return new ControllerLoopQACRO();
    default:
      return nullptr;
    }                                                 
}
