#include "controller_loop.hpp"

#include "modes/controller_loop_NONE.hpp"
#include "modes/controller_loop_QACRO.hpp"
#include "modes/controller_loop_QANGLE.hpp"
#include "modes/controller_loop_QPOS.hpp"
#include "modes/controller_loop_RMANUAL.hpp"
#include "modes/controller_loop_FMANUAL.hpp"

ControllerLoop::ControllerLoop(ControllerMode mode):
    _mode{mode}
{}

void ControllerLoop::job(
  [[maybe_unused]] std::map<std::string,PID>& pids,
  [[maybe_unused]] Control& control,
  [[maybe_unused]] NS& navisys
) 
{
    Eigen::VectorXd vec = applyMixerRotors(0.0,0.0,0.0,0.0);
    control.sendSpeed(vec);
}

ControllerLoop *ControllerLoop::ControllerLoopFactory(ControllerMode mode)
{
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
    case ControllerMode::RMANUAL:
      return new ControllerLoopRMANUAL();
    case ControllerMode::FMANUAL:
      return new ControllerLoopFMANUAL();
    default:
      return nullptr;
    }
}
