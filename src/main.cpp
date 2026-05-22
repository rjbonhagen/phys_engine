#include "phys/math/Vec2.hpp"
#include "phys/Particle.hpp"
#include "Scene.hpp"
#include "Renderer.hpp"
#include <SDL.h>

#include <iostream>

const float PPM = 1.0f; // Pixels / Meter
const int WINDOW_WIDTH = 640;
const int WINDOW_HEIGHT = 480;
const phys::Vec2 ACC_GRAVITY = {0.0f, -9.8f};
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
                        phys::Particle p1{};
                        scene.add_object(p1);
                        p1.position = screen_to_pos({static_cast<phys::real>(e.button.x), static_cast<phys::real>(e.button.y)});
                    }

            }

        }

        scene.step(dt);
        renderer.step(dt);

    }


    return 0;
}


phys::Vec2 worldToScreen(phys::Vec2 posMeters) {
    return posMeters * PPM;
}

phys::Vec2 screen_to_pos(phys::Vec2 p)
{
    return {p.x, static_cast<phys::real>(WINDOW_HEIGHT) - p.y};
}

