#pragma once
#include <string_view>
#include <iostream>

enum ControllerMode
{
    NONE = 0,
    QPOS = 1,
    QANGLE = 2,
    QACRO = 3,
    FMANUAL = 4,
    FACRO = 5,
    FANGLE = 6,

};

constexpr const char* ControllerModeToString(ControllerMode mode) throw()
{
    switch (mode)
    {
    case ControllerMode::NONE:
      return "NONE";
    case ControllerMode::QPOS:
      return "QPOS";
    case ControllerMode::QANGLE:
      return "QANGLE";
    case ControllerMode::QACRO:
      return "QACRO";
    case ControllerMode::FMANUAL:
      return "FMANUAL";
    case ControllerMode::FACRO:
      return "FACRO";
    case ControllerMode::FANGLE:
      return "FANGLE";
    default:
      return "UNKNOWN";
    }
}

constexpr ControllerMode  ControllerModeFromString(const char* mode) throw()
{
  if (std::string_view(mode) == "NONE")
    return ControllerMode::NONE;
  if (std::string_view(mode) == "QPOS")
    return ControllerMode::QPOS;
  if (std::string_view(mode) == "QANGLE")
    return ControllerMode::QANGLE;
  if (std::string_view(mode) == "QACRO")
    return ControllerMode::QACRO;
  if (std::string_view(mode) == "FMANUAL")
    return ControllerMode::FMANUAL;
  if (std::string_view(mode) == "FACRO")
    return ControllerMode::FACRO;
  if (std::string_view(mode) == "FANGLE")
    return ControllerMode::FANGLE;

  std::cerr << "Unknown mode: " << mode << std::endl;
  return ControllerMode::NONE;
}
