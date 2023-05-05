#pragma once

enum ControllerMode
{
    none = 0,
    position = 1,
    angle = 2,
    acro = 3
};

inline const char* ToString(ControllerMode mode)
{
    switch (mode)
    {
        case none: return "None";
        case position: return "Position";
        case angle: return "Angle";
        case acro: return "Acro";
        default: return "Unknown";
    }
}