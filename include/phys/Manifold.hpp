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

        // Impulse accumulated across solver iterations, so each pass applies
        // only the delta. Clamping the total rather than the delta is what
        // keeps a contact from pulling bodies together.
        real normal_impulse{0.0f};
        real tangent_impulse{0.0f};

        // Target separating velocity, fixed before iteration begins. Applying
        // -(1+e)*vn every pass would re-apply restitution on each one.
        real bias{0.0f};

        Manifold(Object* A, Object* B, bool colliding, Vec2 normal, real penetration, Vec2 contact_point) : A{A}, B(B), colliding(colliding), normal(normal), penetration(penetration), contact_point(contact_point) {};
        
    };
    
}
