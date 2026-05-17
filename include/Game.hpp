#pragma once
#include <SDL.h>
#include "phys/Renderer.hpp"
#include "phys/Particle.hpp"
#include <vector>

class Game
{   
    private:
    std::vector<phys::Object> objects{};
    Renderer* RENDERER = nullptr;
    void loop();
    void integrate(phys::Object& o, float dt);

    public:
    Game();
    Game(size_t width, size_t height);
    bool add_particle();



};