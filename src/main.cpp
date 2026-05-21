#include "phys/math/Vec2.hpp"
#include "phys/Particle.hpp"
#include "Scene.hpp"
#include "Renderer.hpp"
#include "SDL.h"

#include <iostream>

const float PPM = 1.0f; // Pixels / Meter
const int WINDOW_WIDTH = 640;
const int WINDOW_HEIGHT = 480;
const phys::Vec2 ACC_GRAVITY = {0, -9.8};
const float dt = 1.0f/60.0f;

phys::Vec2 worldToScreen(phys::Vec2 posMeters);
phys::Vec2 screen_to_pos(phys::Vec2 pixels);

int main(int argc, char* argv[])
{
    Scene scene{};
    Renderer renderer(WINDOW_WIDTH, WINDOW_HEIGHT);


    phys::Particle p{};
    p.position = {WINDOW_WIDTH / 2 , WINDOW_HEIGHT / 2};
    p.forces += ACC_GRAVITY;


    scene.add_object(p);

   
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
                        p.position = screen_to_pos({(phys::real)e.button.x, (phys::real)e.button.y});
                    }

            }

        }

        scene.step(dt);
        renderer.render_circle(worldToScreen(p.position), p.radius, 255, 255, 255);
        renderer.step(dt);


    }


    return 0;
}


phys::Vec2 worldToScreen(phys::Vec2 posMeters) {
    return posMeters * PPM;
}

phys::Vec2 screen_to_pos(phys::Vec2 p)
{
    phys::Vec2 v{p.x + WINDOW_WIDTH, p.y + WINDOW_HEIGHT};
    return v*(-1);
}

