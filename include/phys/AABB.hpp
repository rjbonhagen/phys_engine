#pragma once
#include "phys/Object.hpp"
#include <stdexcept>

namespace phys
{
    struct AABB : Object
    {
        private:
        Vec2 min;
        Vec2 max;
        Vec2 half_body;
        
        public:

<<<<<<< HEAD
        AABB(Vec2 min, Vec2 max, Vec2 velocity, 
            Vec2 acceleration, Vec2 forces, 
            real mass, real restitution)
            : min(min), max(max), half_body((max - min) / 2), 
        Object((min + max) / 2, velocity, acceleration, forces, mass, restitution)
=======
        AABB(Vec2 min, Vec2 max, Vec2 velocity, Vec2 acceleration, Vec2 forces, real mass, real restitution)
        : min(min), max(max), Object((min + max)/ 2, velocity, acceleration, forces, mass, restitution)
>>>>>>> 796c363 (add min max)
         { if (min == max) throw std::invalid_argument("min must not equal max"); }

        Vec2 get_min() const { return position - half_body; }
        Vec2 get_max() const { return position + half_body; }
        Vec2 get_half_body() const { return half_body; }
    };
}