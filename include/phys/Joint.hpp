#pragma once
#include "phys/Object.hpp"

namespace phys
{
    // Holds two anchor points a fixed distance apart. With b null the second
    // anchor is a fixed world point, which is what a pendulum pivot is.
    //
    // Anchors are stored in each body's local frame so they rotate with it;
    // an anchor offset from the centre is what lets a joint apply torque.
    struct Joint
    {
        Object* a{nullptr};
        Object* b{nullptr};

        Vec2 local_a{};
        Vec2 local_b{};     // unused when b is null
        Vec2 world_anchor{};// used when b is null

        real length{0.0f};

        // Accumulated impulse, kept across steps to warm start like a contact.
        real impulse{0.0f};
    };
}
