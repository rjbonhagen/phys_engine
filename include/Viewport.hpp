#pragma once
#include <algorithm>

#include "phys/math/Vec2.hpp"

// Maps between world metres and screen pixels. The world keeps its aspect
// ratio and is centred in the window, so resizing changes only what the view
// looks like, never where anything is in the world.
//
// No SDL, so the transform is directly testable.
class Viewport
{
    public:
    Viewport(int window_w, int window_h, phys::real world_w, phys::real world_h)
    {
        set_window(window_w, window_h);
        set_world(world_w, world_h);
    }

    void set_window(int w, int h)
    {
        window_width  = (w > 1) ? w : 1;
        window_height = (h > 1) ? h : 1;
        update();
    }

    void set_world(phys::real w, phys::real h)
    {
        world_width  = w;
        world_height = h;
        update();
    }

    phys::real pixels_per_metre() const { return ppm; }
    phys::Vec2 world_size() const { return {world_width, world_height}; }

    phys::Vec2 to_screen(phys::Vec2 world) const
    {
        return { origin.x + world.x * ppm, origin.y - world.y * ppm };
    }

    phys::Vec2 to_world(phys::Vec2 screen) const
    {
        if (ppm <= 0.0f) return {0.0f, 0.0f};
        return { (screen.x - origin.x) / ppm, (origin.y - screen.y) / ppm };
    }

    private:
    void update()
    {
        if (world_width <= 0.0f || world_height <= 0.0f) { ppm = 1.0f; origin = {}; return; }

        // One scale for both axes, so a circle stays round.
        ppm = std::min(static_cast<phys::real>(window_width)  / world_width,
                       static_cast<phys::real>(window_height) / world_height);

        // Centre the world, leaving equal letterbox margins on the long axis.
        // The y origin is the screen row of world y = 0, which is the bottom.
        origin = { (static_cast<phys::real>(window_width)  - world_width  * ppm) * 0.5f,
                   (static_cast<phys::real>(window_height) + world_height * ppm) * 0.5f };
    }

    int window_width{1};
    int window_height{1};
    phys::real world_width{1.0f};
    phys::real world_height{1.0f};

    phys::real ppm{1.0f};
    phys::Vec2 origin{};
};
