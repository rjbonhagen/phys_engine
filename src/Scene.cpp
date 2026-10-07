#include "Scene.hpp"
#include <algorithm>
#include <chrono>
#include <functional>
#include <cmath>
#include <memory>
#include <stdexcept>

using namespace phys;

Scene::Scene(real width, real height) : SCENE_WIDTH(width), SCENE_HEIGHT(height)
{

}

void Scene::add_circle(Vec2 p, Vec2 v, Vec2 a, real r, Vec2 f, real m, real rest)
{
    if (m <= 0.0f) throw std::invalid_argument("mass must be positive");
    auto circle = std::make_unique<Circle>(p, v, a, f, m, rest, r);
    objects.push_back(std::move(circle));
}

void Scene::add_aabb(Vec2 min, Vec2 max, Vec2 velocity, Vec2 acceleration, Vec2 forces, real mass, real restitution)
{
    if (mass <= 0.0f) throw std::invalid_argument("mass must be positive");
    auto box = std::make_unique<AABB>(min, max, velocity, acceleration, forces, mass, restitution);
    objects.push_back(std::move(box));
}


void Scene::add_plane(Vec2 point, Vec2 normal, real restitution)
{
    objects.push_back(std::make_unique<Plane>(point, normal, restitution));
}

void Scene::remove_object(size_t index)
{
    if (index >= objects.size()) return;

    // Drop any wall handle pointing at the object about to be destroyed,
    // otherwise reposition_walls() would write through a dangling pointer.
    for (auto*& wall : walls)
        if (wall == objects[index].get()) wall = nullptr;

    objects.erase(objects.begin() + index);
}

void Scene::create_walls(real thickness)
{
    wall_thickness = thickness;

    const Vec2 zero{0.0f, 0.0f};
    for (auto*& wall : walls)
    {
        // Placeholder geometry; reposition_walls() below sets the real extents.
        add_aabb({0.0f, 0.0f}, {1.0f, 1.0f}, zero, zero, zero, INFINITY, 1.0f);
        wall = static_cast<AABB*>(objects.back().get());
    }

    reposition_walls();
}

void Scene::set_dimensions(real w, real h)
{
    SCENE_WIDTH  = w;
    SCENE_HEIGHT = h;
    reposition_walls();
}

void Scene::reposition_walls()
{
    const real t = wall_thickness;
    const real w = SCENE_WIDTH;
    const real h = SCENE_HEIGHT;

    if (walls[0]) walls[0]->resize({  -t,   -t}, {w + t,  0.0f});   // bottom
    if (walls[1]) walls[1]->resize({  -t,    h}, {w + t, h + t});   // top
    if (walls[2]) walls[2]->resize({  -t,   -t}, { 0.0f, h + t});   // left
    if (walls[3]) walls[3]->resize({   w,   -t}, {w + t, h + t});   // right
}

void Scene::integrate(Object& o, real dt)
{
    o.prev_position = o.position;

    o.velocity += o.acceleration * dt;

    // Tunnelling backstop: a body moving further than the thinnest collider in
    // one step passes through it undetected.
    const real speed = o.velocity.length();
    if (speed > MAX_SPEED) o.velocity = o.velocity * (MAX_SPEED / speed);

    o.position += o.velocity * dt;
}

void Scene::resolve_border_collision_circle(Circle& c)
{
    if ( (c.position.x - c.radius) < 0.0f )
    {
        c.position.x = c.radius;
        if (c.velocity.x < 0.0f)
        {
            c.velocity.x *= -c.restitution;
        }
    }
    if ( (c.position.x + c.radius) > SCENE_WIDTH )
    {
        c.position.x = SCENE_WIDTH - c.radius;
        if (c.velocity.x > 0.0f)
        {
            c.velocity.x *= -c.restitution;
        }
    }
    if (c.position.y - c.radius < 0.0f) {
        c.position.y = c.radius;
        if (c.velocity.y < 0.0f)
        {
            c.velocity.y *= -c.restitution;
        }
    }

    if (c.position.y + c.radius > SCENE_HEIGHT) {
        c.position.y = SCENE_HEIGHT - c.radius;
        if (c.velocity.y > 0.0f)
        {
            c.velocity.y *= -c.restitution;
        }
    }

}

// Static bodies are modelled as infinite mass, so they are never integrated and
// never need waking.
static bool is_static(const Object& o) { return !std::isfinite(o.mass); }

int Scene::count_sleeping() const
{
    int n = 0;
    for (const auto& o : objects)
        if (o->asleep && !is_static(*o)) n++;
    return n;
}

// Bodies sleep as an island, not individually. Per-body sleeping cannot work in
// a stack: whatever falls asleep first is pushed by the next contact its awake
// neighbour resolves, which wakes it again. So contacts are unioned into islands
// and an island only sleeps once every member is slow.
//
// Static bodies are left out of the union, otherwise a shared floor would merge
// every stack in the scene into one island that never settles.
void Scene::update_sleep(real dt, const std::vector<Manifold>& manifolds)
{
    const size_t n = objects.size();

    std::unordered_map<const Object*, size_t> index_of;
    index_of.reserve(n);
    for (size_t i = 0; i < n; i++) index_of[objects[i].get()] = i;

    std::vector<size_t> parent(n);
    for (size_t i = 0; i < n; i++) parent[i] = i;

    auto find = [&parent](size_t i)
    {
        while (parent[i] != i) { parent[i] = parent[parent[i]]; i = parent[i]; }
        return i;
    };

    for (const auto& m : manifolds)
    {
        if (!m.colliding || is_static(*m.A) || is_static(*m.B)) continue;

        const size_t a = find(index_of[m.A]);
        const size_t b = find(index_of[m.B]);
        if (a != b) parent[a] = b;
    }

    std::vector<char> island_slow(n, 1);
    for (size_t i = 0; i < n; i++)
    {
        if (is_static(*objects[i])) continue;
        if (objects[i]->velocity.length() > SLEEP_SPEED) island_slow[find(i)] = 0;
    }

    for (size_t i = 0; i < n; i++)
    {
        Object& o = *objects[i];

        if (is_static(o)) { o.asleep = true; continue; }

        if (island_slow[find(i)])
        {
            o.idle_time += dt;
            if (o.idle_time >= SLEEP_DELAY)
            {
                o.asleep   = true;
                o.velocity = {0.0f, 0.0f};
            }
        }
        else
        {
            o.idle_time = 0.0f;
            o.asleep    = false;
        }
    }
}

void Scene::step(real dt)
{
    using clock = std::chrono::steady_clock;
    const auto step_begin = clock::now();

    contacts_skipped_asleep = 0;
    stats = Stats{};

    for (const auto& o : objects)
    {
        if (o->asleep) continue;
        o->acceleration = o->forces / o->mass;
        integrate(*o, dt);
    }

    // Clamp every circle before any pair is measured, so detection never sees
    // a mix of pre- and post-clamp positions.
    for (const auto& o : objects)
    {
        if (auto* circ = dynamic_cast<Circle*>(o.get()))
        {
            resolve_border_collision_circle(*circ);
        }
    }

    std::vector<Manifold> manifolds;

    const auto detect_begin = clock::now();

    for (const auto& o : objects)
    {
        if (auto* circ = dynamic_cast<Circle*>(o.get()))
        {
            circle_handler(*circ, manifolds);
        }
        else if (auto* rect = dynamic_cast<AABB*>(o.get()))
        {
            aabb_handler(*rect, manifolds);
        }
    }

    const auto detect_end = clock::now();
    stats.contacts = manifolds.size();

    for (auto& m : manifolds) prepare_contact(m);

    for (int i = 0; i < solver_iterations; i++)
        for (auto& m : manifolds) solve_velocity(m);

    for (auto& m : manifolds) correct_position(m);

    const auto solve_end = clock::now();

    decltype(contact_cache) next;
    next.reserve(manifolds.size());
    for (const auto& m : manifolds)
        if (m.colliding) next[{m.A, m.B}] = {m.normal_impulse, m.tangent_impulse};
    contact_cache.swap(next);

    last_contacts.clear();
    last_contacts.reserve(manifolds.size());
    for (const auto& m : manifolds)
        if (m.colliding) last_contacts.push_back({m.contact_point, m.normal, m.penetration});

    update_sleep(dt, manifolds);

    using ms = std::chrono::duration<double, std::milli>;
    stats.detect_ms = ms(detect_end - detect_begin).count();
    stats.solve_ms  = ms(solve_end - detect_end).count();
    stats.step_ms   = ms(clock::now() - step_begin).count();
}


void Scene::circle_handler(Circle& c, std::vector<Manifold>& manifolds)
{

    for (const auto& other : objects)
    {
        if (&c == other.get()) continue;

        Vec2 norm{0.0f, 0.0f};
        real penetration = 0.0f;

        if (auto* other_circle = dynamic_cast<Circle*>(other.get()))
        {
            // The pair comes up once per circle; keep one ordering. std::less
            // is defined for unrelated pointers where < is not.
            if (std::less<const Object*>{}(other_circle, &c)) continue;

            stats.candidate_pairs++;
            if (!circle_vs_circle(c, *other_circle, penetration)) continue;

            const Vec2 diff = c.position - other_circle->position;
            norm = (diff.length() > 0.0f) ? diff.normalized() : Vec2{1.0f, 0.0f};
        }
        else if (auto* other_box = dynamic_cast<AABB*>(other.get()))
        {
            stats.candidate_pairs++;
            if (!aabb_vs_circle(*other_box, c, norm, penetration)) continue;
        }
        else if (auto* other_plane = dynamic_cast<Plane*>(other.get()))
        {
            stats.candidate_pairs++;
            if (!circle_vs_plane(*other_plane, c, norm, penetration)) continue;
        }
        else
        {
            continue;
        }

        manifolds.push_back(Manifold(&c, other.get(), true, norm, penetration,
                                           c.position - norm * c.radius));
    }
}


void Scene::aabb_handler(AABB& box, std::vector<Manifold>& manifolds)
{
    for (const auto& other : objects)
    {
        if (&box == other.get()) continue;

        auto* other_box = dynamic_cast<AABB*>(other.get());
        if (!other_box) continue;

        Vec2 norm{0, 0};
        real penetration = 0.0f;
        // Same pair-visited-twice situation as circle vs circle.
        if (std::less<const Object*>{}(other_box, &box)) continue;

        stats.candidate_pairs++;
        if (!aabb_vs_aabb(box, *other_box, norm, penetration)) continue;

        // Centre of the overlap rectangle. This was {0, 0} while nothing read
        // the field, which would have drawn every box contact at the origin.
        const Vec2 lo{std::max(box.get_min().x, other_box->get_min().x),
                      std::max(box.get_min().y, other_box->get_min().y)};
        const Vec2 hi{std::min(box.get_max().x, other_box->get_max().x),
                      std::min(box.get_max().y, other_box->get_max().y)};

        manifolds.push_back(Manifold(&box, other_box, true, norm, penetration, (lo + hi) / 2.0f));
    }
}

// Fixes the restitution target using the approach velocity measured once, before
// any impulse is applied, then warm starts from last step's impulse.
void Scene::prepare_contact(Manifold& m)
{
    if (m.A->asleep && m.B->asleep)
    {
        contacts_skipped_asleep++;
        return;
    }

    const Vec2 v_ab       = m.A->velocity - m.B->velocity;
    const real vel_normal = Vec2::dot(v_ab, m.normal);

    m.bias = (vel_normal < -RESTITUTION_THRESHOLD)
           ? m.A->restitution * m.B->restitution * vel_normal
           : 0.0f;

    const auto cached = contact_cache.find({m.A, m.B});
    m.normal_impulse  = (cached != contact_cache.end()) ? cached->second.normal  : 0.0f;
    m.tangent_impulse = (cached != contact_cache.end()) ? cached->second.tangent : 0.0f;

    if (m.normal_impulse != 0.0f || m.tangent_impulse != 0.0f)
    {
        const Vec2 tangent{-m.normal.y, m.normal.x};
        const Vec2 impulse = m.normal * m.normal_impulse + tangent * m.tangent_impulse;
        m.A->velocity += impulse / m.A->mass;
        m.B->velocity -= impulse / m.B->mass;
    }
}

void Scene::solve_velocity(Manifold& m)
{
    if (!m.colliding) return;
    if (m.A->asleep && m.B->asleep) return;

    Object* A = m.A;
    Object* B = m.B;

    // Guard the denominator, not the impulse: two infinite-mass bodies sum to an
    // inverse mass of exactly zero, and neither can be moved anyway.
    const real inv_mass_sum = 1.0f / A->mass + 1.0f / B->mass;
    if (inv_mass_sum <= 0.0f) return;

    const Vec2 v_ab       = A->velocity - B->velocity;
    const real vel_normal = Vec2::dot(v_ab, m.normal);

    const real j = -(vel_normal + m.bias) / inv_mass_sum;

    // Clamp the running total, not this pass's delta: a contact may only push.
    const real total = std::max(m.normal_impulse + j, 0.0f);
    const real delta = total - m.normal_impulse;
    m.normal_impulse = total;

    const Vec2 impulse = m.normal * delta;
    A->velocity += impulse / A->mass;
    B->velocity -= impulse / B->mass;

    // Friction, against the post-normal-impulse velocity. The limit uses the
    // running normal total, so a contact under more load resists more.
    const Vec2 tangent{-m.normal.y, m.normal.x};
    const real vel_tangent = Vec2::dot(A->velocity - B->velocity, tangent);

    const real jt    = -vel_tangent / inv_mass_sum;
    const real limit = std::sqrt(A->friction * B->friction) * m.normal_impulse;

    const real total_t = std::clamp(m.tangent_impulse + jt, -limit, limit);
    const real delta_t = total_t - m.tangent_impulse;
    m.tangent_impulse  = total_t;

    const Vec2 friction_impulse = tangent * delta_t;
    A->velocity += friction_impulse / A->mass;
    B->velocity -= friction_impulse / B->mass;
}

void Scene::correct_position(Manifold& m)
{
    if (!m.colliding) return;
    if (m.A->asleep && m.B->asleep) return;

    Object* A = m.A;
    Object* B = m.B;

    const real inv_mass_sum = 1.0f / A->mass + 1.0f / B->mass;
    if (inv_mass_sum <= 0.0f) return;

    const real percent = 0.8f;
    const real slop    = 0.01f;

    const real correction_mag = std::max(m.penetration - slop, 0.0f) / inv_mass_sum * percent;
    const Vec2 correction     = m.normal * correction_mag;

    A->position += correction / A->mass;
    B->position -= correction / B->mass;
}

// Signed distance along the plane normal. The normal already points from the
// solid side toward the circle, which is the B-to-A convention the solver wants.
bool Scene::circle_vs_plane(const Plane& p, const Circle& c, Vec2& norm, real& penetration) const
{
    const real distance = Vec2::dot(c.position - p.position, p.normal);
    if (distance > c.radius) return false;

    norm        = p.normal;
    penetration = c.radius - distance;
    return true;
}

bool Scene::circle_vs_circle(const Circle& a, const Circle& b, real& penetration) const
{
    Vec2 diff = a.position - b.position;
    real d = diff.length();
    if (d <= a.radius + b.radius)
    {
        penetration = a.radius + b.radius - d;
        return true;
    }
    else return false;
}

bool Scene::aabb_vs_aabb(const AABB& a, const AABB& b, Vec2& norm, real& penetration) const {
    Vec2 d = b.position - a.position;

    real x_overlap = a.get_half_body().x + b.get_half_body().x - std::fabs(d.x);
    real y_overlap = a.get_half_body().y + b.get_half_body().y - std::fabs(d.y);

    if (x_overlap <= 0 || y_overlap <= 0) return false;

    if (x_overlap < y_overlap)
    {
        norm = (d.x < 0) ? Vec2{1, 0} : Vec2{-1, 0};
        penetration = x_overlap;
    }
    else
    {
        norm = (d.y < 0) ? Vec2{0, 1} : Vec2{0, -1};
        penetration = y_overlap;
    }

    return true;
}

bool Scene::aabb_vs_circle(const AABB& a, const Circle& c, Vec2& norm, real& penetration) const
{
    const Vec2 min = a.get_min();
    const Vec2 max = a.get_max();
    const Vec2 closest{
        std::clamp(c.position.x, min.x, max.x),
        std::clamp(c.position.y, min.y, max.y)
    };
    // Points from the box surface toward the circle, i.e. from B to A.
    const Vec2 to_circle = c.position - closest;
    const real distance  = to_circle.length();

    if (distance > c.radius) return false;

    if (distance > 0.0f)
    {
        norm        = to_circle / distance;
        penetration = c.radius - distance;
        return true;
    }

    // Centre is inside the box. The shortest way out is wrong past the midline
    // -- the nearest face is then the far side, ejecting the circle through the
    // box. Recover the entry face from where it was at the start of the step.
    const Vec2 prev = c.prev_position;

    const real out_left   = min.x - prev.x;   // positive if prev was outside
    const real out_right  = prev.x - max.x;
    const real out_bottom = min.y - prev.y;
    const real out_top    = prev.y - max.y;

    const real crossed = std::max({out_left, out_right, out_bottom, out_top});

    if (crossed > 0.0f)
    {
        if      (crossed == out_left)   norm = {-1.0f,  0.0f};
        else if (crossed == out_right)  norm = { 1.0f,  0.0f};
        else if (crossed == out_bottom) norm = { 0.0f, -1.0f};
        else                            norm = { 0.0f,  1.0f};
    }
    else
    {
        // Already inside at the start of the step, so there is no entry face to
        // recover. Take the shortest way out.
        const real to_left   = c.position.x - min.x;
        const real to_right  = max.x - c.position.x;
        const real to_bottom = c.position.y - min.y;
        const real to_top    = max.y - c.position.y;

        const real nearest = std::min({to_left, to_right, to_bottom, to_top});

        if      (nearest == to_left)   norm = {-1.0f,  0.0f};
        else if (nearest == to_right)  norm = { 1.0f,  0.0f};
        else if (nearest == to_bottom) norm = { 0.0f, -1.0f};
        else                           norm = { 0.0f,  1.0f};
    }

    // Distance to clear the box along norm, plus a radius.
    const real exit_distance = (norm.x < 0.0f) ? (c.position.x - min.x)
                             : (norm.x > 0.0f) ? (max.x - c.position.x)
                             : (norm.y < 0.0f) ? (c.position.y - min.y)
                                               : (max.y - c.position.y);

    penetration = exit_distance + c.radius;
    return true;
}
