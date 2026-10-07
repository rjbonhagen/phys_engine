#pragma once
#include <array>
#include <vector>
#include <memory>
#include <unordered_map>

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

    // Velocity-solver passes per step. One pass cannot propagate a contact
    // through a stack, which is why a single-pass solver sinks.
    int solver_iterations{8};

    // Last step's accumulated normal impulse per contact, used to warm start
    // this step. The handlers order each pair deterministically, so (A, B) is
    // stable across steps and needs no canonicalising. Rebuilt every step, so
    // entries for destroyed objects cannot accumulate.
    struct ContactKey
    {
        const phys::Object* a;
        const phys::Object* b;
        bool operator==(const ContactKey& o) const { return a == o.a && b == o.b; }
    };
    struct ContactKeyHash
    {
        size_t operator()(const ContactKey& k) const
        {
            const size_t h = std::hash<const phys::Object*>{}(k.a);
            return h ^ (std::hash<const phys::Object*>{}(k.b) << 1);
        }
    };
    struct ContactImpulses { phys::real normal{0.0f}; phys::real tangent{0.0f}; };
    std::unordered_map<ContactKey, ContactImpulses, ContactKeyHash> contact_cache;

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
    void prepare_contact(phys::Manifold& m);
    void solve_velocity(phys::Manifold& m);
    void correct_position(phys::Manifold& m);
    void reposition_walls();


    public:
    Scene(phys::real width, phys::real height);
    // Optional: adds four static AABBs enclosing the scene. set_dimensions
    // then keeps them in sync.
    void create_walls(phys::real thickness);
    void set_dimensions(phys::real w, phys::real h);
    void set_solver_iterations(int n) { solver_iterations = (n > 0) ? n : 1; }
    int  get_solver_iterations() const { return solver_iterations; }
    void add_circle(phys::Vec2 p, phys::Vec2 v, phys::Vec2 a, phys::real r, phys::Vec2 f, phys::real m, phys::real rest);
    void add_aabb(phys::Vec2 min, phys::Vec2 max, phys::Vec2 velocity, phys::Vec2 acceleration, phys::Vec2 forces, phys::real mass, phys::real restitution);
    void remove_object(size_t index);
    void step(phys::real dt);
    std::vector<std::unique_ptr<phys::Object>>& get_objects() { return objects; }
};