#pragma once
#include <SDL.h>
#include <vector>
#include <memory>

#include "phys/math/Vec2.hpp"
#include "phys/Object.hpp"
#include "phys/Circle.hpp"
#include "phys/AABB.hpp"


class Renderer
{

    private:
    SDL_Window* WINDOW = nullptr;
    SDL_Renderer* RENDERER = nullptr;
    const int WINDOW_WIDTH;
    const int WINDOW_HEIGHT;
    const phys::real PPM;
    phys::Vec2 position_to_screen(phys::Vec2 p);
    void render_objects(const std::vector<std::unique_ptr<phys::Object>>& objects, int selected_idx);
    void update_title(phys::real dt);


    public:
    Renderer(int width, int height, phys::real ppm);
    ~Renderer();
    SDL_Window*   get_window()   const { return WINDOW; }
    SDL_Renderer* get_renderer() const { return RENDERER; }
    void render_circle(phys::Vec2 p, phys::real r, bool highlight = false);
    void render_rectangle(phys::Vec2 min, phys::Vec2 max, bool highlight = false);
    void render_arrow(phys::Vec2 from, phys::Vec2 to, SDL_Color color);
    void step(phys::real dt, const std::vector<std::unique_ptr<phys::Object>>& objects, int selected_idx = -1);
    void present();


};
