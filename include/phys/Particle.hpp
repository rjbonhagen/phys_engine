#pragma once
#include "math/Vec2.hpp"
#include "Object.hpp"
#include "math/Real.hpp"

namespace phys
{
    struct Particle : Object
    {
        real radius{10};
    };
    
}
