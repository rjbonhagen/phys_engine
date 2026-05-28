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
        real penetration;
        Vec2 contact_point;

        Manifold(Object* A, Object* B, bool colliding, Vec2 normal, real penetration, Vec2 contact_point) : A{A}, B(B), colliding(colliding), normal(normal), penetration(penetration), contact_point(contact_point) {};
        
    };
    
}
