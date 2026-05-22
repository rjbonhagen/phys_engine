#pragma once
#include "phys/math/Vec2.hpp"

namespace phys
{

    struct Object
    {
        Vec2 position{};
        Vec2 velocity{};
        Vec2 acceleration{};

        Vec2 forces{};

        real mass{};

        virtual ~Object() = default;
    };


}
