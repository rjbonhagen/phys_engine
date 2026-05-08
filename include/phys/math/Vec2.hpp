#pragma once
#include "Real.hpp"

namespace phys
{
    class Vec2
    {
    public:
        real x;
        real y;

        Vec2(real x = 0.0f, real y = 0.0f) : x(x), y(y) {}
    };
}
