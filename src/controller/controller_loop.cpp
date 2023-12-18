#include "controller_loop.hpp"

#include "modes/controller_loop_NONE.hpp"
#include "modes/controller_loop_QACRO.hpp"
#include "modes/controller_loop_QANGLE.hpp"
#include "modes/controller_loop_QPOS.hpp"
#include "modes/controller_loop_FMANUAL.hpp"
#include "modes/controller_loop_FACRO.hpp"
#include "modes/controller_loop_FANGLE.hpp"
#include "modes/controller_loop_RMANUAL.hpp"
#include "modes/controller_loop_RAUTOLAUNCH.hpp"
#include "modes/controller_loop_RANGLE.hpp"
#include "modes/controller_loop_RGUIDED.hpp"

ControllerLoop::ControllerLoop(ControllerMode mode):
    _mode{mode}
{}

void ControllerLoop::job(
  [[maybe_unused]] std::map<std::string,std::unique_ptr<Controller>>& controllers,
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
    case ControllerMode::FMANUAL:
      return new ControllerLoopFMANUAL();
    case ControllerMode::FACRO:
      return new ControllerLoopFACRO();
    case ControllerMode::FANGLE:
      return new ControllerLoopFANGLE();
    case ControllerMode::RAUTOLAUNCH:
      return new ControllerLoopRAUTOLAUNCH();
    case ControllerMode::RMANUAL:
      return new ControllerLoopRMANUAL();
    case ControllerMode::RANGLE:
      return new ControllerLoopRANGLE();
    case ControllerMode::RGUIDED:
      return new ControllerLoopRGUIDED();
    default:
      return nullptr;
    }
}

bool ControllerLoop::checkJoystickLength(const Eigen::VectorXd& joystick, const int minimalSize)
{
    if(joystick.size() < minimalSize)
    {
        std::cerr << "Mode " << _mode << " requires at least " << minimalSize << "input axis!";
        return false;
    }
    return true;
}
