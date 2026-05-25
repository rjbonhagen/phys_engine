#pragma once
#include "phys/Object.hpp"

namespace phys
{
    struct Circle : Object
    {
        real radius;

        Circle(Vec2 p = {0,0}, Vec2 v = {0,0}, Vec2 a = {0,0}, Vec2 f = {0,0}, real m = 0, real r = 0) : Object{p, v, a, f, m}, radius(r) {}

    };
}
