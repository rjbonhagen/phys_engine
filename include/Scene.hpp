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
    phys::real SCENE_WIDTH;
    phys::real SCENE_HEIGHT;
    std::vector<std::unique_ptr<phys::Object>> objects{};

    void integrate(phys::Object& o, phys::real dt);
    void resolve_border_collision_circle(phys::Circle& c);
    bool circle_vs_circle(const phys::Circle& a, const phys::Circle& b) const;
    bool aabb_vs_aabb(const phys::AABB& a, const phys::AABB& b, phys::Vec2& norm, phys::real& penetration) const;
    void resolve_collision(phys::Manifold& m);


    public:
    Scene(phys::real width, phys::real height);
    void set_dimensions(phys::real w, phys::real h) { SCENE_WIDTH = w; SCENE_HEIGHT = h; }
    void add_circle(phys::Vec2 p, phys::Vec2 v, phys::Vec2 a, phys::real r, phys::Vec2 f, phys::real m, phys::real rest);
    void add_aabb(phys::Vec2 min, phys::Vec2 max, phys::Vec2 velocity, phys::Vec2 acceleration, phys::Vec2 forces, phys::real mass, phys::real restitution);
    void remove_object(size_t index);
    void step(phys::real dt);
    std::vector<std::unique_ptr<phys::Object>>& get_objects() { return objects; }
};