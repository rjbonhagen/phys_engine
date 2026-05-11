#pragma once
#include "math/Vec2.hpp"

namespace phys
{
    struct Particle
    {
        Vec2 position{};
        Vec2 velocity{};
        Vec2 acceleration{};

        Vec2 forceAccum {};

        float mass {1.0f};


    };
    
}
