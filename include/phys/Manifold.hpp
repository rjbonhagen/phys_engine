#pragma once
#include "phys/Object.hpp"

namespace phys
{
    struct Manifold
    {
        Object* A;
        Object* B;

        bool colliding;
        Vec2 normal;
        float penetration;
        Vec2 contact_point;

        Manifold(Object* a, Object* b, bool colliding, Vec2 normal, float penetration, Vec2 contact_point) : A(a), B(b), colliding(colliding)
        
    }
    
}
