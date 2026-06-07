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

        AABB(Vec2 min, Vec2 max, Vec2 velocity, Vec2 acceleration, Vec2 forces, real mass, real restitution)
        : min(min), max(max), half_body((max - min) / 2), Object((min + max) / 2, velocity, acceleration, forces, mass, restitution)
         { 
            if (min == max) throw std::invalid_argument("min must not equal max"); 
        }

        Vec2 get_min() const { return position - half_body; }
        Vec2 get_max() const { return position + half_body; }
        Vec2 get_half_body() const { return half_body; }

        void resize(Vec2 new_min, Vec2 new_max)
        {
            half_body = (new_max - new_min) / 2;
            position  = new_min + half_body;
        }
    };
}