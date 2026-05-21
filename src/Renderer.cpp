#include "Renderer.hpp"

Renderer::Renderer(size_t width, size_t height) : WINDOW_WIDTH(width), WINDOW_HEIGHT(height)
{
    if ( SDL_Init(SDL_INIT_EVERYTHING) < 0 )
    {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "SDL init error: %s", SDL_GetError());
        return;
    }
    
    if ( SDL_CreateWindowAndRenderer(WINDOW_WIDTH, WINDOW_HEIGHT, 0, &WINDOW, &RENDERER) < 0)
    {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Window and renderer creation error: %s", SDL_GetError());
        return;
    }


}

Renderer::~Renderer()
{
    SDL_DestroyRenderer( RENDERER );
    SDL_DestroyWindow( WINDOW );
    SDL_Quit();
}

void Renderer::render_circle(phys::Vec2 p, int r, int red, int blue, int green)
{
    p = position_to_screen(p);

    SDL_SetRenderDrawColor(RENDERER, red, blue, green, 255);
    int x = 0, y = r, d = 1 - r;
    while (x <= y)
    {
        int success = 0;

        SDL_RenderDrawLine(RENDERER, p.x - x, p.y + y, p.x + x, p.y + y);
        SDL_RenderDrawLine(RENDERER, p.x - x, p.y - y, p.x + x, p.y - y);
        SDL_RenderDrawLine(RENDERER, p.x - y, p.y + x, p.x + y, p.y + x);
        SDL_RenderDrawLine(RENDERER, p.x - y, p.y - x, p.x + y, p.y - x);

        if (success < 0) {  SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Window and renderer creation error: %s", SDL_GetError()); }

        if (d < 0) d += 2 * x + 3;
        else       d += 2 * (x - y--) + 5;
        x++;
    }
}

void Renderer::step(float dt)
{
    SDL_RenderPresent( RENDERER );
    SDL_SetRenderDrawColor(RENDERER, 0, 0, 0, 255);
    SDL_RenderClear( RENDERER );
    SDL_Delay(dt*100);
}

phys::Vec2 Renderer::position_to_screen(phys::Vec2 p)
{
    phys::Vec2 v{p.x - WINDOW_WIDTH, p.y - WINDOW_HEIGHT};
    return v*(-1);
}