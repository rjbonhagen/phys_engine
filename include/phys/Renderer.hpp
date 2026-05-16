#include <SDL.h>

class Renderer
{
    SDL_Window* WINDOW = nullptr;    
    SDL_Renderer* RENDERER = nullptr;
    const size_t WINDOW_WIDTH;
    const size_t WINDOW_HEIGHT;

    Renderer(size_t width, size_t height) : WINDOW_WIDTH(width), WINDOW_HEIGHT(height) {};

};

