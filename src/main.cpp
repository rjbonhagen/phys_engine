#include "phys/math/Vec2.hpp"
#include "Scene.hpp"
#include "Renderer.hpp"
#include <SDL.h>

#include <iostream>

const int PPM = 50; // Pixels / Meter
const int WINDOW_WIDTH = 640;
const int WINDOW_HEIGHT = 480;
const phys::Vec2 ACC_GRAVITY = {0.0f, -9.8f};
phys::Vec2 screen_to_pos(phys::Vec2 pixels);

int main(int argc, char* argv[])
{
    Scene scene{(int)(WINDOW_WIDTH / PPM), (int)(WINDOW_HEIGHT / PPM)};
    Renderer renderer(WINDOW_WIDTH, WINDOW_HEIGHT, PPM);

    Uint64 prev = SDL_GetPerformanceCounter();
    bool quit = false;
    while ( !quit )
    {
        
        SDL_Event e;
        while ( SDL_PollEvent( &e ) != 0) 
        {
            switch ( e.type )
            {
                case SDL_QUIT:
                    return 0;
                case SDL_MOUSEBUTTONDOWN:
                    if (e.button.button == SDL_BUTTON_LEFT)
                    {
                        phys::Vec2 p = screen_to_pos({static_cast<phys::real>(e.button.x), static_cast<phys::real>(e.button.y)}) / PPM;
                        scene.add_circle(p, {0,0}, {0,0}, .2f, ACC_GRAVITY * 0.001f, 0.001f, 1.0f);
                    }
                    if (e.button.button == SDL_BUTTON_RIGHT)
                    {
                        phys::Vec2 p = screen_to_pos({static_cast<phys::real>(e.button.x), static_cast<phys::real>(e.button.y)}) / PPM;
                        scene.add_circle(p, {0,0}, {0,0}, 1.0f, ACC_GRAVITY * 1.0f, 1.0f, .5f);
                    }

            }

        }
        Uint64 now = SDL_GetPerformanceCounter();
        float dt = (now - prev) / (float)SDL_GetPerformanceFrequency();
        prev = now;

        scene.step(dt);
        renderer.step(dt, scene.get_objects());

    }


    return 0;
}

phys::Vec2 screen_to_pos(phys::Vec2 p)
{
    return {p.x, static_cast<phys::real>(WINDOW_HEIGHT) - p.y};
}
