#include "params.hpp"
#include <iostream>

Params* Params::_singleton = nullptr;

Params::Params() 
{
    if(_singleton != nullptr)
    {
        std::cerr << "Only one instance of Params should exist";
        return;
    }
    _singleton = this;

    STEP_TIME = 0.001;
}

Params::~Params() 
{
    _singleton = nullptr;
}

const Params *Params::getSingleton() 
{    
    if(_singleton == nullptr)
    {
        std::cerr << "Params sigleton is null!" << std::endl;
    }
    return _singleton;
}