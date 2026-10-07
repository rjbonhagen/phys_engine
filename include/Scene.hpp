#pragma once
#include <array>
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

    // Hard ceiling on speed, in world units per second. Keeps per-step
    // displacement below the thickness of the thinnest wall.
    static constexpr phys::real MAX_SPEED = 50.0f;

    // Non-owning, ordered bottom, top, left, right. Null until create_walls();
    // remove_object() clears any entry it erases.
    std::array<phys::AABB*, 4> walls{};
    phys::real wall_thickness{0.0f};

    void integrate(phys::Object& o, phys::real dt);
    void resolve_border_collision_circle(phys::Circle& c);
    void circle_handler(phys::Circle& c, std::vector<phys::Manifold>& manifolds);
    void aabb_handler(phys::AABB& box, std::vector<phys::Manifold>& manifolds);
    bool circle_vs_circle(const phys::Circle& a, const phys::Circle& b, phys::real& penetration) const;
    bool aabb_vs_aabb(const phys::AABB& a, const phys::AABB& b, phys::Vec2& norm, phys::real& penetration) const;
    bool aabb_vs_circle(const phys::AABB& a, const phys::Circle& c, phys::Vec2& norm, phys::real& penetration) const;
    void resolve_collision(phys::Manifold& m);
    void reposition_walls();


    public:
    Scene(phys::real width, phys::real height);
    // Optional: adds four static AABBs enclosing the scene. set_dimensions
    // then keeps them in sync.
    void create_walls(phys::real thickness);
    void set_dimensions(phys::real w, phys::real h);
    void add_circle(phys::Vec2 p, phys::Vec2 v, phys::Vec2 a, phys::real r, phys::Vec2 f, phys::real m, phys::real rest);
    void add_aabb(phys::Vec2 min, phys::Vec2 max, phys::Vec2 velocity, phys::Vec2 acceleration, phys::Vec2 forces, phys::real mass, phys::real restitution);
    void remove_object(size_t index);
    void step(phys::real dt);
    std::vector<std::unique_ptr<phys::Object>>& get_objects() { return objects; }
};