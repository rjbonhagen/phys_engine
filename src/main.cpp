#include "phys/math/Vec2.hpp"
#include "phys/Particle.hpp"
#include "SDL.h"
#include <print>


SDL_Window* WINDOW = nullptr;    
SDL_Surface* WIN_SURFACE = nullptr;

bool sdl_init();
bool sdl_load();
void integrate(phys::Particle& p, float dt);

int main(int argc, char* argv[])
{


    if ( !sdl_init() ) return 1;
    if ( !sdl_load() ) return 1;





    SDL_DestroyWindow(WINDOW);
    SDL_Quit();
    return 0;
}


bool sdl_init()
{
    
    if (SDL_Init(SDL_INIT_EVERYTHING) < 0)
    {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "SDL init error: %s", SDL_GetError());
        return false;
    }

    WINDOW = SDL_CreateWindow("Engine", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 1280, 720, SDL_WINDOW_SHOWN);

    if ( !WINDOW )
    {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Error creating window: %s", SDL_GetError());
        return 1;
    }

        WIN_SURFACE = SDL_GetWindowSurface(WINDOW);

    if ( !WIN_SURFACE )
    {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Error getting window surface: %s", SDL_GetError());
        return 1;
    }

    return true;
}

bool sdl_load()
{

    SDL_FillRect(WIN_SURFACE, NULL, SDL_MapRGB(WIN_SURFACE->format, 255, 255, 255));
    SDL_UpdateWindowSurface(WINDOW);
    SDL_Log("Window ready");    

}

bool sdl_loop()
{
    SDL_Event e;
    while ( SDL_PollEvent(&e) )
    {
        switch (e.type)
            case SDL_QUIT:
                return false;
            

    }

}



void integrate(phys::Particle& p, float dt)
{
    p.acceleration = p.forceAccum / p.mass;
    p.velocity += p.acceleration * dt;
    p.position += p.velocity * dt;
}