#pragma once
#include "phys/Circle.hpp"

namespace phys
{
    struct Particle : Circle
    {

        Particle () { mass = .001f ; radius = .01f; }
    };
    
}
