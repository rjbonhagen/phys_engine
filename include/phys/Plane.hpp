#pragma once
#include <cmath>
#include <stdexcept>

#include "phys/Object.hpp"

namespace phys
{
    // A static half-space: everything on the negative side of the normal is
    // solid. This is how an inclined surface is represented, since a a box was axis-aligned and cannot be one.
    struct Plane : Object
    {
        Vec2 normal;

        Plane(Vec2 point, Vec2 plane_normal, real restitution = 0.2f)
            : Object(point, Vec2{}, Vec2{}, Vec2{}, INFINITY, restitution), normal(plane_normal)
        {
            const real length = plane_normal.length();
            if (length == 0.0f) throw std::invalid_argument("plane normal must be non-zero");
            normal = plane_normal / length;
        }
    };
}
