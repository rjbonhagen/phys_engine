#pragma once
#include "phys/math/Vec2.hpp"

namespace phys
{

    struct Object
    {
        Vec2 position;
        Vec2 velocity;
        Vec2 acceleration;

        Vec2 forces;

        real mass;
        real restitution{0.5f};

        Object() : position(), velocity(), acceleration(), forces(), mass(0.0f), restitution(0.5f) {}
        Object(Vec2 position, Vec2 velocity, Vec2 acceleration, Vec2 forces, real mass, real restitution = 0.5f)
            : position(position), velocity(velocity), acceleration(acceleration), forces(forces), mass(mass), restitution(restitution) {}

        virtual ~Object() = default;
        bool operator==(const Object& o) const { return (position == o.position) && 
                                                        (velocity == o.velocity) && 
                                                        (acceleration == o.acceleration) && 
                                                        (forces == o.forces) && 
                                                        (mass == o.mass); };
        bool operator!=(const Object& o) const { return !(*this == o); }
    };


}
