#pragma once
#include <SDL.h>
#include <vector>
#include <memory>

#include "phys/math/Real.hpp"
#include "phys/Circle.hpp"
#include "phys/Manifold.hpp"


class Scene
{   
    private:
    const int SCENE_WIDTH;
    const int SCENE_HEIGHT;
    std::vector<std::unique_ptr<phys::Object>> objects{};

    void integrate(phys::Object& o, phys::real dt);
    void resolve_border_collision_circle(phys::Circle& c);
    bool circle_vs_circle(const phys::Circle& a, const phys::Circle& b) const;
    void resolve_collision(phys::Manifold& m);


    public:
    Scene(int width, int height);
    void add_circle(phys::Vec2 p, phys::Vec2 v, phys::Vec2 a, phys::real r, phys::Vec2 f, phys::real m, phys::real rest);
    void step(phys::real dt);
    std::vector<std::unique_ptr<phys::Object>>& get_objects() { return objects; }
};