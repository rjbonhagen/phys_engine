#pragma once
#include <SDL.h>
#include <vector>
#include <memory>

#include "Renderer.hpp"
#include "phys/Particle.hpp"
#include "phys/math/Real.hpp"
#include "phys/Circle.hpp"


class Scene
{   
    private:
    const int SCENE_WIDTH;
    const int SCENE_HEIGHT;
    inline static std::vector<std::unique_ptr<phys::Object>> objects{};

    void integrate(phys::Object& o, float dt);
    void resolve_border_collision_circle(phys::Object& c, phys::real restitution);
    bool circle_vs_circle(const phys::Circle& a, const phys::Circle& b) const;


    public:
    Scene(int width, int height);
    void add_particle(phys::Vec2 p, phys::Vec2 v, phys::Vec2 a, phys::Vec2 f, phys::real m);
    void step(phys::real dt);
    static std::vector<std::unique_ptr<phys::Object>>& get_objects() { return objects; }
};