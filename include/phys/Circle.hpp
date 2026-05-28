#pragma once
#include "phys/Object.hpp"

namespace phys
{
    struct Circle : Object
    {
        real radius;

        Circle(Vec2 position = {}, Vec2 velocity = {}, Vec2 acceleration = {}, Vec2 forces = {}, real mass = 0.0f, real restitution = 0.5f, real radius = 0.0f)
            : Object(position, velocity, acceleration, forces, mass, restitution), radius(radius) {}

    };
}
