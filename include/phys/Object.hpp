#pragma once
#include "phys/math/Vec2.hpp"

namespace phys
{

    struct Object
    {
        Vec2 position{};
        Vec2 velocity{};
        Vec2 acceleration{};

        Vec2 forces{};

        real mass{};

        Object(Vec2 p = {0,0}, Vec2 v = {0,0}, Vec2 a = {0,0}, Vec2 f = {0,0}, real m = 0) : position(p), velocity(v), acceleration(a), forces(f), mass(m) {};
        virtual ~Object() = default;
        bool operator==(const Object& o) const { return (position == o.position) && 
                                                        (velocity == o.velocity) && 
                                                        (acceleration == o.acceleration) && 
                                                        (forces == o.forces) && 
                                                        (mass == o.mass); };
        bool operator!=(const Object& o) const { return !(*this == o); }
    };


}
