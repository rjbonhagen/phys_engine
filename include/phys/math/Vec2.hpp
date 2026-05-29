#pragma once

#include <cmath>
#include <cassert>
#include "Real.hpp"

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
        void operator*=(const Vec2& v) { x *= v.x; y *= v.y ;}
        void operator/=(const Vec2& v) { x /= v.x; y /= v.y ;}

        Vec2 operator*(const real c) const { return {x * c, y * c}; }
        Vec2 operator/(const real c) const 
        { 
            assert( c != 0.0f);
            return {x / c, y / c}; 
        } 

        bool operator==(const Vec2& v) const { return (x == v.x) && (y == v.y); }
        bool operator!=(const Vec2& v) const { return !(*this == v); }

        bool operator<(const Vec2& v) const { return length() < v.length(); }
        bool operator<=(const Vec2& v) const { return (length() < v.length()) || (*this == v); }
        bool operator>(const Vec2& v) const {return length() > v.length(); }
        bool operator>=(const Vec2& v) const {return (length() > v.length()) || (*this == v); }

        real length() const { return std::sqrt(x*x + y*y); }
 
        void normalize()
        {
            real l = length();
            assert (l != 0.0f);
            x /= l;
            y /= l;
        }

        Vec2 normalized() const 
        {
            real l = length();
            assert (l != 0.0f);
            return {x / l, y / l};
        }

        static real dot(const Vec2& v, const Vec2& v2) { return (v.x*v2.x + v.y*v2.y); }

        


    };
}
 