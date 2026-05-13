#include "phys/math/Vec2.hpp"
#include "phys/Particle.hpp"
#include "SDL.h"
#include <print>

int main(int argc, char* argv[])
{
    SDL_Surface* s = nullptr;
    SDL_Window* w = nullptr;
    w = SDL_CreateWindow("Engine", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 1280, 720, SDL_WINDOW_SHOWN);

    if (!w)
    {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Error creating window: %s", SDL_GetError());
        return 1;
    }

    s = SDL_GetWindowSurface(w);

    if (!s)
    {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Error getting window surface: %s", SDL_GetError());
        return 1;
    }

    SDL_FillRect(s, NULL, SDL_MapRGB(s->format, 255, 90, 120));
    SDL_UpdateWindowSurface(w);
    SDL_Log("Window ready");
    std::print("hello");

    SDL_Event e;
    bool running = true;
    while (running)
    {
        while (SDL_PollEvent(&e))
        {
            if (e.type == SDL_QUIT)
                running = false;
        }
    }

    SDL_DestroyWindow(w);
    SDL_Quit();
    return 0;
}

bool init()
{
    
    if (SDL_Init(SDL_INIT_EVERYTHING) < 0)
    {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "SDL init error: %s", SDL_GetError());
        return false;
    }

    return true;

}


void Integrate(phys::Particle& p, float dt)
{
    p.acceleration = p.forceAccum / p.mass;
    p.velocity += p.acceleration * dt;
    p.position += p.velocity * dt;
}