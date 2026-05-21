#pragma once
#include <SDL.h>
#include "Renderer.hpp"
#include "phys/Particle.hpp"
#include <vector>
#include "phys/math/Real.hpp"

class Scene
{   
    private:
    std::vector<phys::Object*> objects{};

    void integrate(phys::Object* o, float dt);

    public:
    bool add_object(phys::Object& o);
    void step(phys::real dt);



};