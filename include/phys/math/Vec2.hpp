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

        Vec2() : x(0.0f), y(0.0f) {}
        Vec2(real x, real y) : x(x), y(y) {}

        Vec2 operator+(const Vec2& v) const { return {x + v.x, y + v.y}; }
        Vec2 operator-(const Vec2& v) const { return {x - v.x, y - v.y}; }

        void operator+=(const Vec2& v) { x += v.x; y += v.y; }
        void operator-=(const Vec2& v) { x -= v.x; y -= v.y; }
        // Component-wise. Prefer the scalar overloads below for plain scaling.
        void operator*=(const Vec2& v) { x *= v.x; y *= v.y; }
        void operator/=(const Vec2& v)
        {
            assert(v.x != 0.0f && v.y != 0.0f);
            x /= v.x; y /= v.y;
        }

        // Scalar, matching operator* / operator/ below.
        void operator*=(const real c) { x *= c; y *= c; }
        void operator/=(const real c)
        {
            assert(c != 0.0f);
            x /= c; y /= c;
        }

        Vec2 operator*(const real c) const { return {x * c, y * c}; }
        Vec2 operator/(const real c) const 
        { 
            assert( c != 0.0f);
            return {x / c, y / c}; 
        } 

        bool operator==(const Vec2& v) const { return (x == v.x) && (y == v.y); }
        bool operator!=(const Vec2& v) const { return !(*this == v); }

        // Magnitude only, so distinct vectors of equal length compare equal.
        // That makes this a partial order -- not valid for std::sort or std::map.
        bool operator<(const Vec2& v)  const { return length() <  v.length(); }
        bool operator<=(const Vec2& v) const { return length() <= v.length(); }
        bool operator>(const Vec2& v)  const { return length() >  v.length(); }
        bool operator>=(const Vec2& v) const { return length() >= v.length(); }

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

        // 2D cross product is a scalar: the z component of the 3D cross. Used
        // as the torque a force at an offset applies about the centre.
        static real cross(const Vec2& r, const Vec2& f) { return r.x*f.y - r.y*f.x; }

        // Rotated 90 degrees. cross(omega, r) in 2D is omega * r.perpendicular().
        Vec2 perpendicular() const { return {-y, x}; }

        


    };
}
 