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
    size_t WINDOW_WIDTH;
    size_t WINDOW_HEIGHT;
    phys::Vec2 position_to_screen(phys::Vec2 p);
    void render_objects();


    public:
    Renderer(size_t width, size_t height);
    ~Renderer();
    void render_circle(phys::Vec2, int r, int red, int blue, int green);
    void render_velocity(phys::Object& o);
    void step(float dt);


};

