#pragma once
#include <SDL.h>

class Renderer
{

    private:
    SDL_Window* WINDOW = nullptr;    
    SDL_Renderer* RENDERER = nullptr;
    size_t WINDOW_WIDTH;
    size_t WINDOW_HEIGHT;
    public:
    Renderer(size_t width = 640, size_t height = 480);
    ~Renderer();
    void render_circle(int cx, int cy, int r, int red, int blue, int green);
    void step();


};

