#pragma once
#include "Real.hpp"
#include <cmath>

namespace phys
{
    struct Vec2
    {
        real x;
        real y;

        Vec2(real x = 0.0f, real y = 0.0f) : x(x), y(y) {}

        Vec2 operator+(const Vec2& v) const { return {x + v.x, y + v.y}; }
        Vec2 operator-(const Vec2& v) const { return {x - v.x, y - v.y}; }

        void operator+=(const Vec2& v) { x += v.x; y += v.y; }
        void operator-=(const Vec2& v) { x -= v.x; y -= v.y; }

        Vec2 operator*(const real c) const { return {x * c, y * c}; }
        Vec2 operator/(const real c) const { return {x / c, y / c}; }

        real length() const { return std::sqrt(x*x + y*y); }

        void normalize()
        {
            real l = length();
            x /= l;
            y /= l;
        }

        Vec2 normalized() const 
        {
            real l = length();
            return {x / l, y / l};
        }

        real dot(const Vec2& v) const { return (x*v.x + y*v.y); }

        


    };
}
