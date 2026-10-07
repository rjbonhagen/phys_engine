#pragma once
#include <cmath>

#include "phys/Object.hpp"

namespace phys
{
    struct Circle : Object
    {
        real radius;

        Circle(Vec2 position, Vec2 velocity, Vec2 acceleration, Vec2 forces, real mass, real restitution, real radius)
            : Object(position, velocity, acceleration, forces, mass, restitution), radius(radius)
        {
            // Solid disc about its centre: I = m r^2 / 2. A static disc gets
            // zero inverse inertia, same as its inverse mass.
            const real inertia = 0.5f * mass * radius * radius;
            inv_inertia = (inertia > 0.0f && std::isfinite(inertia)) ? 1.0f / inertia : 0.0f;
        }

    };
}
