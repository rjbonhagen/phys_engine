#pragma once
#include <SDL.h>

#include "phys/math/Vec2.hpp"
#include "phys/Object.hpp"
#include "Scene.hpp"


class Renderer
{

    private:
    SDL_Window* WINDOW = nullptr;    
    SDL_Renderer* RENDERER = nullptr;
    const int WINDOW_WIDTH;
    const int WINDOW_HEIGHT;
    const phys::real PPM;
    phys::Vec2 position_to_screen(phys::Vec2 p);
    void render_objects();
    void update_title(phys::real dt);


    public:
    Renderer(int width, int height, phys::real ppm);
    ~Renderer();
    void render_circle(phys::Vec2 p, phys::real r);
    void render_velocity(phys::Object& o);
    void step(phys::real dt);


};

