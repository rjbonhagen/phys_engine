#pragma once
#include <SDL.h>
#include <vector>
#include <memory>

#include "Viewport.hpp"
#include "phys/math/Vec2.hpp"
#include "phys/Object.hpp"
#include "phys/Circle.hpp"
#include "phys/Box.hpp"
#include "phys/Plane.hpp"
#include "phys/Manifold.hpp"
#include "phys/Joint.hpp"


class Renderer
{

    private:
    bool valid = false;
    SDL_Window* WINDOW = nullptr;
    SDL_Renderer* RENDERER = nullptr;

    // The window fits the world rather than the world tracking the window, so
    // resizing cannot change simulation behaviour.
    Viewport view;

    phys::Vec2 position_to_screen(phys::Vec2 p) const { return view.to_screen(p); }
    void render_objects(const std::vector<std::unique_ptr<phys::Object>>& objects, int selected_idx);
    void update_title(phys::real dt);


    public:
    Renderer(int width, int height, phys::real world_w, phys::real world_h);

    // Call when the window changes size, and when the world does.
    void on_resize(int width, int height);
    void set_world_size(phys::real world_w, phys::real world_h);

    // The one place the screen/world transform lives. main used to carry its
    // own copy, which silently disagreed once the scale stopped being constant.
    phys::Vec2 screen_to_world(phys::real sx, phys::real sy) const { return view.to_world({sx, sy}); }
    phys::real pixels_per_metre() const { return view.pixels_per_metre(); }
    ~Renderer();
    // Check before use: the handles are null if init failed.
    bool is_valid() const { return valid; }
    SDL_Window*   get_window()   const { return WINDOW; }
    SDL_Renderer* get_renderer() const { return RENDERER; }
    void render_circle(phys::Vec2 p, phys::real r, bool highlight = false);
    void render_rectangle(phys::Vec2 min, phys::Vec2 max, bool highlight = false);
    void render_box(const phys::Box& box, bool highlight = false);
    void render_arrow(phys::Vec2 from, phys::Vec2 to, SDL_Color color);
    void render_plane(phys::Vec2 point, phys::Vec2 normal, bool highlight = false);
    void render_contacts(const std::vector<phys::ContactPoint>& contacts);
    void render_joints(const std::vector<phys::Joint>& joints);
    void step(phys::real dt, const std::vector<std::unique_ptr<phys::Object>>& objects, int selected_idx = -1);
    void present();


};
