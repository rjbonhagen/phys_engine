#pragma once
#include "phys/Object.hpp"
#include <stdexcept>

namespace phys
{
    struct Box : Object
    {
        private:
        // The box is stored as its centre (Object::position) plus a half extent.
        // Corners are derived, so they stay correct as the box moves.
        Vec2 half_body;

        public:

        Box(Vec2 min, Vec2 max, Vec2 velocity, Vec2 acceleration, Vec2 forces, real mass, real restitution)
        : Object((min + max) / 2, velocity, acceleration, forces, mass, restitution),
          half_body((max - min) / 2)
        {
            // An inverted box would yield a negative half extent, which makes
            // get_min()/get_max() swap and every overlap test misbehave.
            if (min.x >= max.x || min.y >= max.y)
                throw std::invalid_argument("Box requires min < max on both axes");
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