#include "phys/math/Vec2.hpp"
#include "phys/Particle.hpp"
#include "SDL.h"


bool sdl_init();
bool sdl_loop();
void sdl_kill();
void integrate(phys::Particle& p, float dt);
void render_circle(SDL_Renderer* renderer, int cx, int cy, int r);

int main(int argc, char* argv[])
{
    if ( !sdl_init() ) return 1;

    phys::Particle p{};

    
    while ( sdl_loop() )
    {
        SDL_Delay(10);
    }

    sdl_kill();
    return 0;
}


bool sdl_init()
{
    
    if ( SDL_Init(SDL_INIT_EVERYTHING) < 0 )
    {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "SDL init error: %s", SDL_GetError());
        return false;
    }
    
    if ( SDL_CreateWindowAndRenderer(WINDOW_WIDTH, WINDOW_HEIGHT, 0, &WINDOW, &RENDERER) < 0)
    {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Window and renderer creation error: %s", SDL_GetError());
        return false;
    }




    return true;
}

bool sdl_loop()
{
    SDL_Event e;
    while ( SDL_PollEvent(&e) )
    {
        switch (e.type)
            case SDL_QUIT:
                return false;
                break;

    }


    SDL_SetRenderDrawColor( RENDERER, 0, 0, 0, 255 );
    SDL_RenderClear( RENDERER );

    SDL_SetRenderDrawColor( RENDERER, 255, 255, 255, 255 );
    render_circle(RENDERER, WINDOW_WIDTH/2, WINDOW_HEIGHT/2, 25);
    SDL_RenderPresent( RENDERER );

    return true;

}

void sdl_kill()
{
    SDL_DestroyRenderer( RENDERER );
    SDL_DestroyWindow( WINDOW );
    SDL_Quit();

}

void render_circle(SDL_Renderer* renderer, int cx, int cy, int r)
{
    int x = 0, y = r, d = 1 - r;
    while (x <= y)
    {
        SDL_RenderDrawLine(renderer, cx - x, cy + y, cx + x, cy + y);
        SDL_RenderDrawLine(renderer, cx - x, cy - y, cx + x, cy - y);
        SDL_RenderDrawLine(renderer, cx - y, cy + x, cx + y, cy + x);
        SDL_RenderDrawLine(renderer, cx - y, cy - x, cx + y, cy - x);

        if (d < 0) d += 2 * x + 3;
        else       d += 2 * (x - y--) + 5;
        x++;
    }
}


void integrate(phys::Particle& p, float dt)
{
    p.acceleration = p.forceAccum / p.mass;
    p.velocity += p.acceleration * dt;
    p.position += p.velocity * dt;
}