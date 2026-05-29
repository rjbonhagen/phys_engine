#pragma once
#include "phys/Object.hpp"

namespace phys
{
    struct Circle : Object
    {
        real radius;

        Circle(Vec2 position, Vec2 velocity, Vec2 acceleration, Vec2 forces, real mass, real restitution = 0.5f, real radius)
            : Object(position, velocity, acceleration, forces, mass, restitution), radius(radius) {}

    };
}
