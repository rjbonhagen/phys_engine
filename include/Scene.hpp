#pragma once
#include <array>
#include <utility>
#include <vector>
#include <memory>
#include <unordered_map>

#include "phys/math/Real.hpp"
#include "phys/Circle.hpp"
#include "phys/Manifold.hpp"
#include "phys/Box.hpp"
#include "phys/Plane.hpp"
#include "phys/Joint.hpp"


class Scene
{
    public:
    // Per-step measurements. Timings are wall clock for the last step only, so
    // a benchmark should average over many.
    struct Stats
    {
        size_t candidate_pairs{0};   // pairs the broad phase handed to narrow
        size_t contacts{0};          // pairs that actually overlapped
        double detect_ms{0.0};
        double solve_ms{0.0};
        double step_ms{0.0};
        size_t toi_clamps{0};     // bodies stopped at a swept time of impact
    };

    enum class BroadPhase { AllPairs, SpatialHash };

    private:
    phys::real SCENE_WIDTH;
    phys::real SCENE_HEIGHT;
    std::vector<std::unique_ptr<phys::Object>> objects{};

    // Overlap left behind when a body is stopped at a time of impact, so the
    // discrete pass afterwards sees a contact. Parked at exactly zero distance
    // it can slip through on the following step instead.
    static constexpr phys::real TOI_SKIN = 0.01f;

    // Velocity-solver passes per step. One pass cannot propagate a contact
    // through a stack, which is why a single-pass solver sinks.
    int solver_iterations{8};

    // Below this approach speed a contact gets no restitution. A resting body
    // re-approaches by one gravity increment every step, and bouncing that back
    // is what makes a settled stack jitter forever.
    static constexpr phys::real RESTITUTION_THRESHOLD = 0.5f;

    // A body below SLEEP_SPEED for SLEEP_DELAY seconds goes to sleep.
    static constexpr phys::real SLEEP_SPEED = 0.05f;
    static constexpr phys::real SLEEP_DELAY = 0.5f;
    static constexpr phys::real SLEEP_SPIN  = 0.1f;

    size_t contacts_skipped_asleep{0};
    std::vector<phys::ContactPoint> last_contacts{};

    // Which bodies touched last step. last_contacts is pointer-free for the
    // renderer; this keeps the pairs so wake() can reach a body's neighbours.
    std::vector<std::pair<phys::Object*, phys::Object*>> last_pairs{};

    std::vector<phys::Joint> joints{};
    Stats stats{};
    BroadPhase broad_phase{BroadPhase::SpatialHash};

    void update_sleep(phys::real dt, const std::vector<phys::Manifold>& manifolds);

    // Last step's accumulated normal impulse per contact, used to warm start
    // this step. The handlers order each pair deterministically, so (A, B) is
    // stable across steps and needs no canonicalising. Rebuilt every step, so
    // entries for destroyed objects cannot accumulate.
    struct ContactKey
    {
        const phys::Object* a;
        const phys::Object* b;
        int index;   // face contacts carry two points; they warm start separately
        bool operator==(const ContactKey& o) const
        {
            return a == o.a && b == o.b && index == o.index;
        }
    };
    struct ContactKeyHash
    {
        size_t operator()(const ContactKey& k) const
        {
            const size_t h = std::hash<const phys::Object*>{}(k.a);
            return h ^ (std::hash<const phys::Object*>{}(k.b) << 1)
                     ^ (static_cast<size_t>(k.index) << 17);
        }
    };
    struct ContactImpulses { phys::real normal{0.0f}; phys::real tangent{0.0f}; };
    std::unordered_map<ContactKey, ContactImpulses, ContactKeyHash> contact_cache;

    // Non-owning, ordered bottom, top, left, right. Null until create_walls();
    // remove_object() clears any entry it erases.
    std::array<phys::Box*, 4> walls{};
    phys::real wall_thickness{0.0f};

    void integrate(phys::Object& o, phys::real dt);
    void resolve_border_collision_circle(phys::Circle& c);
    std::vector<std::pair<size_t, size_t>> broad_phase_all_pairs() const;
    std::vector<std::pair<size_t, size_t>> broad_phase_hash() const;
    void narrow_phase(phys::Object& x, phys::Object& y, std::vector<phys::Manifold>& out);
    void push_box_plane_contacts(const phys::Plane& p, phys::Box& b, std::vector<phys::Manifold>& out);
    bool circle_vs_circle(const phys::Circle& a, const phys::Circle& b, phys::real& penetration) const;
    bool box_vs_box(const phys::Box& a, const phys::Box& b, phys::Vec2& norm, phys::real& penetration) const;
    int  box_vs_box_contacts(const phys::Box& a, const phys::Box& b, phys::Vec2& norm,
                             phys::Vec2 points[2], phys::real depths[2]) const;
    bool box_vs_circle(const phys::Box& a, const phys::Circle& c, phys::Vec2& norm, phys::real& penetration) const;
    bool circle_vs_plane(const phys::Plane& p, const phys::Circle& c, phys::Vec2& norm, phys::real& penetration) const;
    bool box_vs_plane(const phys::Plane& p, const phys::Box& b, phys::Vec2& norm, phys::real& penetration) const;
    int  box_vs_plane_contacts(const phys::Plane& p, const phys::Box& b, phys::Vec2& norm,
                               phys::Vec2 points[2], phys::real depths[2]) const;
    bool swept_circle_vs_plane(const phys::Plane& p, const phys::Circle& c,
                               phys::Vec2 displacement, phys::real& toi) const;
    bool swept_circle_vs_box(const phys::Box& b, const phys::Circle& c,
                              phys::Vec2 displacement, phys::real& toi) const;
    void resolve_tunnelling();
    void prepare_contact(phys::Manifold& m);
    void solve_velocity(phys::Manifold& m);
    void correct_position(phys::Manifold& m);
    void wake_neighbour(phys::Object& o);
    void solve_joint(phys::Joint& j, phys::real dt);
    void warm_start_joint(phys::Joint& j);
    void reposition_walls();


    public:
    Scene(phys::real width, phys::real height);
    // Optional: adds four static Boxs enclosing the scene. set_dimensions
    // then keeps them in sync.
    void create_walls(phys::real thickness);
    void set_dimensions(phys::real w, phys::real h);
    void set_solver_iterations(int n) { solver_iterations = (n > 0) ? n : 1; }
    int  get_solver_iterations() const { return solver_iterations; }

    // Contacts the last step skipped because both bodies were asleep.
    size_t get_contacts_skipped_asleep() const { return contacts_skipped_asleep; }
    int count_sleeping() const;

    // Contacts detected on the last step, for debug rendering.
    const std::vector<phys::ContactPoint>& get_contacts() const { return last_contacts; }

    const Stats& get_stats() const { return stats; }

    void set_broad_phase(BroadPhase bp) { broad_phase = bp; }
    BroadPhase get_broad_phase() const { return broad_phase; }

    // The pairs the current broad phase would hand to the narrow phase. Exposed
    // so a test can check the grid against the all-pairs reference.
    std::vector<std::pair<size_t, size_t>> candidate_pairs() const;
    void add_circle(phys::Vec2 p, phys::Vec2 v, phys::Vec2 a, phys::real r, phys::Vec2 f, phys::real m, phys::real rest);
    void add_box(phys::Vec2 min, phys::Vec2 max, phys::Vec2 velocity, phys::Vec2 acceleration, phys::Vec2 forces, phys::real mass, phys::real restitution);
    // A static inclined surface. The normal points out of the solid side.
    void add_plane(phys::Vec2 point, phys::Vec2 normal, phys::real restitution);
    void remove_object(size_t index);

    // Wakes a body, its resting island, and anything it was touching. Call
    // after moving a body by hand: a sleeping body skips integration, so
    // whatever was resting on it would otherwise hang in the air.
    void wake(size_t index);
    void wake(phys::Object& o);

    // Distance joints. Anchors are given in world space and converted to each
    // body's local frame, so they rotate with it. Pass one index for a joint
    // pinned to a fixed world point.
    // Rest length defaults to the anchor separation at creation, which is what
    // placing a joint in an editor means. Pass one explicitly for a rope or rod
    // of a chosen length.
    void add_joint(size_t a, size_t b, phys::Vec2 anchor_a, phys::Vec2 anchor_b);
    void add_joint(size_t a, size_t b, phys::Vec2 anchor_a, phys::Vec2 anchor_b, phys::real length);
    void add_joint(size_t a, phys::Vec2 anchor_on_body, phys::Vec2 world_anchor);
    void add_joint(size_t a, phys::Vec2 anchor_on_body, phys::Vec2 world_anchor, phys::real length);
    void remove_joints_touching(const phys::Object& o);
    const std::vector<phys::Joint>& get_joints() const { return joints; }
    void step(phys::real dt);
    std::vector<std::unique_ptr<phys::Object>>& get_objects() { return objects; }
};