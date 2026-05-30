#pragma once
#include <vector>
#include <memory>

#include "phys/math/Real.hpp"
#include "phys/Circle.hpp"
#include "phys/Manifold.hpp"
#include "phys/AABB.hpp"


class Scene
{   
    private:
    const int SCENE_WIDTH;
    const int SCENE_HEIGHT;
    std::vector<std::unique_ptr<phys::Object>> objects{};

    void integrate(phys::Object& o, phys::real dt);
    void resolve_border_collision_circle(phys::Circle& c);
    bool circle_vs_circle(const phys::Circle& a, const phys::Circle& b) const;
    bool aabb_vs_aabb(const phys::AABB& a, const phys::AABB& b, phys::Vec2& norm) const;
    void resolve_collision(phys::Manifold& m);


    public:
    Scene(int width, int height);
    void add_circle(phys::Vec2 p, phys::Vec2 v, phys::Vec2 a, phys::real r, phys::Vec2 f, phys::real m, phys::real rest);
    void add_aabb(phys::Vec2 min, phys::Vec2 max, phys::Vec2 velocity, phys::Vec2 acceleration, phys::Vec2 forces, phys::real mass, phys::real restitution);
    void step(phys::real dt);
    std::vector<std::unique_ptr<phys::Object>>& get_objects() { return objects; }
};