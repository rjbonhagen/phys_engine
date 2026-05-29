#pragma once
#include "phys/Object.hpp"
#include <stdexcept>

namespace phys
{
    struct AABB : public Object
    {
    public:
        Vec2 min;
        Vec2 max;

        AABB(Vec2 min, Vec2 max, Vec2 velocity, Vec2 acceleration, Vec2 forces, real mass, real restitution) 
        : min(min), max(max), Object(min, velocity, acceleration, forces, mass, restitution)
         { if (min == max) throw std::invalid_argument("min must not equal max"); }

    };
}