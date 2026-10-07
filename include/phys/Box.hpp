#pragma once
#include <array>
#include <cmath>
#include <stdexcept>

#include "phys/Object.hpp"

namespace phys
{
    // An oriented box: centre (Object::position) plus a half extent, rotated by
    // Object::orientation. At orientation zero it is axis-aligned and behaves
    // exactly as the old AABB did.
    struct Box : Object
    {
        private:
        Vec2 half_body;

        public:

        Box(Vec2 min, Vec2 max, Vec2 velocity, Vec2 acceleration, Vec2 forces, real mass, real restitution)
        : Object((min + max) / 2, velocity, acceleration, forces, mass, restitution),
          half_body((max - min) / 2)
        {
            // An inverted box would yield a negative half extent, which makes
            // every extent and overlap test misbehave.
            if (min.x >= max.x || min.y >= max.y)
                throw std::invalid_argument("Box requires min < max on both axes");

            set_inertia();
        }

        Vec2 get_half_body() const { return half_body; }

        // The box's own axes in world space: column 0 is its local x.
        Vec2 axis_x() const { return { std::cos(orientation),  std::sin(orientation) }; }
        Vec2 axis_y() const { return { -std::sin(orientation), std::cos(orientation) }; }

        std::array<Vec2, 4> corners() const
        {
            const Vec2 ex = axis_x() * half_body.x;
            const Vec2 ey = axis_y() * half_body.y;

            return { position - ex - ey, position + ex - ey,
                     position + ex + ey, position - ex + ey };
        }

        // Half extent of the *world-space bounding box*, which grows as the box
        // rotates. Used by the broad phase, which wants bounds rather than the
        // exact shape.
        Vec2 bounds_half() const
        {
            const Vec2 ax = axis_x();
            const Vec2 ay = axis_y();

            return { std::fabs(ax.x) * half_body.x + std::fabs(ay.x) * half_body.y,
                     std::fabs(ax.y) * half_body.x + std::fabs(ay.y) * half_body.y };
        }

        // Bounding-box corners, not the box's own corners. These coincide only
        // while the orientation is zero.
        Vec2 get_min() const { return position - bounds_half(); }
        Vec2 get_max() const { return position + bounds_half(); }

        void resize(Vec2 new_min, Vec2 new_max)
        {
            half_body = (new_max - new_min) / 2;
            position  = new_min + half_body;
            set_inertia();
        }

        private:
        void set_inertia()
        {
            // Rectangle about its centre: I = m (w^2 + h^2) / 12.
            const real w = half_body.x * 2.0f;
            const real h = half_body.y * 2.0f;
            const real inertia = mass * (w * w + h * h) / 12.0f;

            inv_inertia = (inertia > 0.0f && std::isfinite(inertia)) ? 1.0f / inertia : 0.0f;
        }
    };
}
