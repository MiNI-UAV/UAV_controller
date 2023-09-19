#pragma once

inline double circularError(double demanded, double val)
{
    double diff = demanded-val;
    if(diff > std::numbers::pi) return -2*std::numbers::pi + diff;
    if(diff < -std::numbers::pi) return +2*std::numbers::pi + diff;
    return diff;
}

inline double clampAngle(double angle)
{
    angle = std::fmod(angle + std::numbers::pi,2*std::numbers::pi);
    if (angle < 0)
        angle += 2*std::numbers::pi;
    return angle - std::numbers::pi;
}