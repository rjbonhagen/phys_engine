#pragma once
#include "phys/math/Vec2.hpp"

namespace phys
{

    struct Object
    {
        Vec2 position;

        // Written by Scene::integrate; collision uses it to recover which face
        // a deeply-penetrating body entered through. Excluded from operator==.
        Vec2 prev_position;

        Vec2 velocity;
        Vec2 acceleration;

        Vec2 forces;

        // Defaults to 1: Scene::step computes forces / mass every frame, so a
        // zero default would divide by zero on any directly-constructed Object.
        real mass;
        real restitution{0.5f};

        // Coulomb coefficient. Mixed between two bodies as the geometric mean,
        // so one frictionless body makes the whole contact frictionless.
        real friction{0.3f};

        // A sleeping body skips integration and its contacts skip the solver.
        // Managed by Scene::step; see the sleep limitations noted there.
        bool asleep{false};
        real idle_time{0.0f};

        Object() : position(), prev_position(), velocity(), acceleration(), forces(), mass(1.0f), restitution(0.5f) {}
        Object(Vec2 position, Vec2 velocity, Vec2 acceleration, Vec2 forces, real mass, real restitution = 0.5f)
            : position(position), prev_position(position), velocity(velocity), acceleration(acceleration), forces(forces), mass(mass), restitution(restitution) {}

        virtual ~Object() = default;
        bool operator==(const Object& o) const { return (position == o.position) && 
                                                        (velocity == o.velocity) && 
                                                        (acceleration == o.acceleration) && 
                                                        (forces == o.forces) && 
                                                        (mass == o.mass); };
        bool operator!=(const Object& o) const { return !(*this == o); }
    };


}
