#pragma once
#include "math/Vec2.hpp"

namespace phys
{

    struct Object
    {
        Vec2 position{};
        Vec2 velocity{};
        Vec2 acceleration{};

        Vec2 forceAccum{};

        real mass{1.0f};

    };


}
