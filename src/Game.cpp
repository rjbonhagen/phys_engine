#include "Game.hpp"


Game::Game() 
{
    RENDERER = new Renderer();
    loop();
}

Game::Game(size_t width, size_t height)
{
    RENDERER = new Renderer(width, height);
    loop();
}

bool Game::add_particle()
{
    objects.push_back( phys::Particle{} );
    
}

void Game::loop()
{
    bool quit = false;
    while ( !quit )
    {
        SDL_Event e;
        while ( SDL_PollEvent( &e ) != 0 )
        {
            switch (e.type)
            {
                case SDL_QUIT:
                    quit = true;
                    return;
            }
        }

        
    }
}

void Game::integrate(phys::Object& o, float dt)
{
    o.acceleration = o.forceAccum / o.mass;
    o.velocity += o.acceleration * dt;
    o.position += o.velocity * dt;
}