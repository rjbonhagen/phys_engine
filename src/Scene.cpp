#include "Scene.hpp"
#include <algorithm>
#include <chrono>
#include <limits>
#include <unordered_map>
#include <functional>
#include <cmath>
#include <memory>
#include <stdexcept>

using namespace phys;

// Defined further down, used before it.
static real project_box(const Box& box, const Vec2& axis);
static bool is_static(const Object& o);

Scene::Scene(real width, real height) : SCENE_WIDTH(width), SCENE_HEIGHT(height)
{

}

void Scene::add_circle(Vec2 p, Vec2 v, Vec2 a, real r, Vec2 f, real m, real rest)
{
    if (m <= 0.0f) throw std::invalid_argument("mass must be positive");
    auto circle = std::make_unique<Circle>(p, v, a, f, m, rest, r);
    objects.push_back(std::move(circle));
}

void Scene::add_box(Vec2 min, Vec2 max, Vec2 velocity, Vec2 acceleration, Vec2 forces, real mass, real restitution)
{
    if (mass <= 0.0f) throw std::invalid_argument("mass must be positive");
    auto box = std::make_unique<Box>(min, max, velocity, acceleration, forces, mass, restitution);
    objects.push_back(std::move(box));
}


void Scene::add_plane(Vec2 point, Vec2 normal, real restitution)
{
    objects.push_back(std::make_unique<Plane>(point, normal, restitution));
}

// World point to a body's local frame, and back.
static Vec2 to_local(const Object& o, Vec2 world)
{
    const Vec2 d{world.x - o.position.x, world.y - o.position.y};
    const real c = std::cos(o.orientation);
    const real s = std::sin(o.orientation);
    return { d.x * c + d.y * s, -d.x * s + d.y * c };
}

static Vec2 to_world(const Object& o, Vec2 local)
{
    const real c = std::cos(o.orientation);
    const real s = std::sin(o.orientation);
    return { o.position.x + local.x * c - local.y * s,
             o.position.y + local.x * s + local.y * c };
}

void Scene::add_joint(size_t a, size_t b, Vec2 anchor_a, Vec2 anchor_b)
{
    add_joint(a, b, anchor_a, anchor_b, (anchor_a - anchor_b).length());
}

void Scene::add_joint(size_t a, size_t b, Vec2 anchor_a, Vec2 anchor_b, real length)
{
    if (a >= objects.size() || b >= objects.size() || a == b) return;
    if (!(length >= 0.0f)) return;

    Joint j;
    j.a       = objects[a].get();
    j.b       = objects[b].get();
    j.local_a = to_local(*j.a, anchor_a);
    j.local_b = to_local(*j.b, anchor_b);
    j.length  = length;

    joints.push_back(j);
}

void Scene::add_joint(size_t a, Vec2 anchor_on_body, Vec2 world_anchor)
{
    add_joint(a, anchor_on_body, world_anchor, (anchor_on_body - world_anchor).length());
}

void Scene::add_joint(size_t a, Vec2 anchor_on_body, Vec2 world_anchor, real length)
{
    if (a >= objects.size()) return;
    if (!(length >= 0.0f)) return;

    Joint j;
    j.a            = objects[a].get();
    j.b            = nullptr;
    j.local_a      = to_local(*j.a, anchor_on_body);
    j.world_anchor = world_anchor;
    j.length       = length;

    joints.push_back(j);
}

void Scene::remove_joints_touching(const Object& o)
{
    joints.erase(std::remove_if(joints.begin(), joints.end(),
                                [&o](const Joint& j) { return j.a == &o || j.b == &o; }),
                 joints.end());
}

void Scene::warm_start_joint(Joint& j)
{
    if (!j.a) return;

    const Vec2 pa = to_world(*j.a, j.local_a);
    const Vec2 pb = j.b ? to_world(*j.b, j.local_b) : j.world_anchor;

    const Vec2 d = pa - pb;
    const real distance = d.length();
    if (distance < 1e-6f) return;

    const Vec2 n  = d / distance;
    const Vec2 ra = pa - j.a->position;

    const Vec2 applied = n * j.impulse;
    j.a->velocity += applied / j.a->mass;
    j.a->angular_velocity += j.a->inv_inertia * Vec2::cross(ra, applied);

    if (j.b)
    {
        const Vec2 rb = pb - j.b->position;
        j.b->velocity -= applied / j.b->mass;
        j.b->angular_velocity -= j.b->inv_inertia * Vec2::cross(rb, applied);
    }
}

void Scene::solve_joint(Joint& j, real dt)
{
    if (!j.a || dt <= 0.0f) return;

    const Vec2 pa = to_world(*j.a, j.local_a);
    const Vec2 pb = j.b ? to_world(*j.b, j.local_b) : j.world_anchor;

    const Vec2 d = pa - pb;
    const real distance = d.length();
    if (distance < 1e-6f) return;

    const Vec2 n  = d / distance;
    const Vec2 ra = pa - j.a->position;
    const Vec2 rb = j.b ? pb - j.b->position : Vec2{};

    const real inv_mass_a = 1.0f / j.a->mass;
    const real inv_mass_b = j.b ? 1.0f / j.b->mass : 0.0f;

    const real rn_a = Vec2::cross(ra, n);
    const real rn_b = j.b ? Vec2::cross(rb, n) : 0.0f;

    const real k = inv_mass_a + inv_mass_b
                 + j.a->inv_inertia * rn_a * rn_a
                 + (j.b ? j.b->inv_inertia * rn_b * rn_b : 0.0f);
    if (k <= 0.0f) return;

    const Vec2 va = j.a->velocity + ra.perpendicular() * j.a->angular_velocity;
    const Vec2 vb = j.b ? j.b->velocity + rb.perpendicular() * j.b->angular_velocity
                        : Vec2{};

    const real vn = Vec2::dot(va - vb, n);

    // Baumgarte: feed a fraction of the length error back as velocity, so the
    // joint recovers from drift instead of accumulating it.
    const real beta = 0.2f;
    const real bias = (beta / dt) * (distance - j.length);

    const real magnitude = -(vn + bias) / k;
    j.impulse += magnitude;

    const Vec2 applied = n * magnitude;
    j.a->velocity += applied * inv_mass_a;
    j.a->angular_velocity += j.a->inv_inertia * Vec2::cross(ra, applied);

    if (j.b)
    {
        j.b->velocity -= applied * inv_mass_b;
        j.b->angular_velocity -= j.b->inv_inertia * Vec2::cross(rb, applied);
    }
}

void Scene::wake(size_t index)
{
    if (index < objects.size()) wake(*objects[index]);
}

void Scene::wake(Object& o)
{
    auto rouse = [](Object& body)
    {
        if (is_static(body)) return;
        body.asleep    = false;
        body.idle_time = 0.0f;
    };

    rouse(o);

    // The island only connects dynamic bodies, so dragging a static wall needs
    // the contact pairs as well to reach whatever was resting on it.
    const size_t island = o.island;
    for (const auto& other : objects)
        if (!is_static(*other) && other->island == island) rouse(*other);

    for (const auto& [a, b] : last_pairs)
    {
        if (a == &o && b) wake_neighbour(*b);
        if (b == &o && a) wake_neighbour(*a);
    }
}

// One hop only: a woken body moves, which wakes its own neighbours next step.
void Scene::wake_neighbour(Object& o)
{
    if (is_static(o)) return;

    const size_t island = o.island;
    for (const auto& other : objects)
        if (!is_static(*other) && other->island == island)
        {
            other->asleep    = false;
            other->idle_time = 0.0f;
        }
}

void Scene::remove_object(size_t index)
{
    if (index >= objects.size()) return;

    // Anything resting on this must start falling rather than hang in the air.
    wake(*objects[index]);
    remove_joints_touching(*objects[index]);

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
        add_box({0.0f, 0.0f}, {1.0f, 1.0f}, zero, zero, zero, INFINITY, 1.0f);
        wall = static_cast<Box*>(objects.back().get());
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
    o.position += o.velocity * dt;

    o.angular_velocity += o.torque * o.inv_inertia * dt;
    o.orientation      += o.angular_velocity * dt;
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

// Time of impact of a moving circle against a static plane, as a fraction of
// this step's displacement. Exact: the signed distance is linear along the
// segment.
bool Scene::swept_circle_vs_plane(const Plane& p, const Circle& c,
                                  Vec2 displacement, real& toi) const
{
    const real d0 = Vec2::dot(c.prev_position - p.position, p.normal);
    const real d1 = Vec2::dot(c.prev_position + displacement - p.position, p.normal);

    if (d0 <= c.radius) return false;   // already in contact; discrete handles it
    if (d1 > c.radius)  return false;   // never reaches the surface

    toi = (d0 - c.radius) / (d0 - d1);
    return toi >= 0.0f && toi <= 1.0f;
}

// Slab test against the box grown by the circle radius. The true Minkowski sum
// has rounded corners, so this reports a hit marginally early near one -- which
// errs toward stopping the body rather than letting it through.
bool Scene::swept_circle_vs_box(const Box& b, const Circle& c,
                                 Vec2 displacement, real& toi) const
{
    const Vec2 lo{b.get_min().x - c.radius, b.get_min().y - c.radius};
    const Vec2 hi{b.get_max().x + c.radius, b.get_max().y + c.radius};

    const Vec2 start = c.prev_position;
    real tmin = 0.0f;
    real tmax = 1.0f;

    const real s[2] = {start.x, start.y};
    const real d[2] = {displacement.x, displacement.y};
    const real l[2] = {lo.x, lo.y};
    const real h[2] = {hi.x, hi.y};

    for (int axis = 0; axis < 2; axis++)
    {
        if (std::fabs(d[axis]) < 1e-8f)
        {
            if (s[axis] < l[axis] || s[axis] > h[axis]) return false;
            continue;
        }

        real t1 = (l[axis] - s[axis]) / d[axis];
        real t2 = (h[axis] - s[axis]) / d[axis];
        if (t1 > t2) std::swap(t1, t2);

        tmin = std::max(tmin, t1);
        tmax = std::min(tmax, t2);
        if (tmin > tmax) return false;
    }

    toi = tmin;
    // tmin of zero means the segment began already overlapping.
    return toi > 0.0f && toi <= 1.0f;
}

// Stops a fast body at its first impact with static geometry, which is what
// lets the speed clamp go. Only static bodies are swept: two fast dynamic
// bodies can still pass through each other.
void Scene::resolve_tunnelling()
{
    for (const auto& o : objects)
    {
        auto* c = dynamic_cast<Circle*>(o.get());
        if (!c || c->asleep) continue;

        const Vec2 displacement = c->position - c->prev_position;
        const real travel       = displacement.length();

        // Under one radius the discrete pass cannot miss: the circle still
        // overlaps whatever it reached.
        if (travel <= c->radius) continue;

        real earliest = 1.0f;

        for (const auto& other : objects)
        {
            if (other.get() == static_cast<Object*>(c)) continue;
            if (!is_static(*other)) continue;

            real t = 1.0f;
            bool hit = false;

            if (const auto* pl = dynamic_cast<const Plane*>(other.get()))
                hit = swept_circle_vs_plane(*pl, *c, displacement, t);
            else if (const auto* bx = dynamic_cast<const Box*>(other.get()))
                hit = swept_circle_vs_box(*bx, *c, displacement, t);

            if (hit && t < earliest) earliest = t;
        }

        if (earliest < 1.0f)
        {
            const Vec2 direction = displacement / travel;
            c->position = c->prev_position + displacement * earliest + direction * TOI_SKIN;
            stats.toi_clamps++;
        }
    }
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

    // Jointed bodies share an island too, so a swinging partner keeps the whole
    // assembly awake rather than half of it freezing.
    for (const auto& j : joints)
    {
        if (!j.a || !j.b || is_static(*j.a) || is_static(*j.b)) continue;

        const size_t a = find(index_of[j.a]);
        const size_t b = find(index_of[j.b]);
        if (a != b) parent[a] = b;
    }

    for (size_t i = 0; i < n; i++) objects[i]->island = find(i);

    std::vector<char> island_slow(n, 1);
    for (size_t i = 0; i < n; i++)
    {
        if (is_static(*objects[i])) continue;
        const real spin = std::fabs(objects[i]->angular_velocity);
        if (objects[i]->velocity.length() > SLEEP_SPEED || spin > SLEEP_SPIN)
            island_slow[find(i)] = 0;
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
                o.asleep           = true;
                o.velocity         = {0.0f, 0.0f};
                o.angular_velocity = 0.0f;
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

    resolve_tunnelling();

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

    const auto pairs = candidate_pairs();
    stats.candidate_pairs = pairs.size();

    for (const auto& [i, j] : pairs) narrow_phase(*objects[i], *objects[j], manifolds);

    const auto detect_end = clock::now();
    stats.contacts = manifolds.size();

    for (auto& m : manifolds) prepare_contact(m);

    for (auto& j : joints) { j.impulse = 0.0f; warm_start_joint(j); }

    for (int i = 0; i < solver_iterations; i++)
    {
        for (auto& m : manifolds) solve_velocity(m);
        for (auto& j : joints)    solve_joint(j, dt);
    }

    for (auto& m : manifolds) correct_position(m);

    const auto solve_end = clock::now();

    decltype(contact_cache) next;
    next.reserve(manifolds.size());
    for (const auto& m : manifolds)
        if (m.colliding) next[{m.A, m.B, m.point_index}] = {m.normal_impulse, m.tangent_impulse};
    contact_cache.swap(next);

    last_contacts.clear();
    last_contacts.reserve(manifolds.size());
    last_pairs.clear();
    last_pairs.reserve(manifolds.size());

    for (const auto& m : manifolds)
        if (m.colliding)
        {
            last_contacts.push_back({m.contact_point, m.normal, m.penetration});
            last_pairs.push_back({m.A, m.B});
        }

    update_sleep(dt, manifolds);

    using ms = std::chrono::duration<double, std::milli>;
    stats.detect_ms = ms(detect_end - detect_begin).count();
    stats.solve_ms  = ms(solve_end - detect_end).count();
    stats.step_ms   = ms(clock::now() - step_begin).count();
}


std::vector<std::pair<size_t, size_t>> Scene::broad_phase_all_pairs() const
{
    const size_t n = objects.size();

    std::vector<std::pair<size_t, size_t>> pairs;
    if (n > 1) pairs.reserve(n * (n - 1) / 2);

    for (size_t i = 0; i < n; i++)
        for (size_t j = i + 1; j < n; j++)
            pairs.emplace_back(i, j);

    return pairs;
}

// Axis-aligned bounds of a finite body. Planes are infinite half-spaces and
// have none, which is why they cannot be bucketed.
static bool body_bounds(const Object& o, Vec2& lo, Vec2& hi)
{
    if (const auto* c = dynamic_cast<const Circle*>(&o))
    {
        lo = {c->position.x - c->radius, c->position.y - c->radius};
        hi = {c->position.x + c->radius, c->position.y + c->radius};
        return true;
    }
    if (const auto* b = dynamic_cast<const Box*>(&o))
    {
        lo = b->get_min();
        hi = b->get_max();
        return true;
    }
    return false;
}

std::vector<std::pair<size_t, size_t>> Scene::broad_phase_hash() const
{
    const size_t n = objects.size();

    std::vector<size_t> planes;
    real extent_sum = 0.0f;
    size_t finite = 0;

    for (size_t i = 0; i < n; i++)
    {
        Vec2 lo, hi;
        if (!body_bounds(*objects[i], lo, hi)) { planes.push_back(i); continue; }
        extent_sum += std::max(hi.x - lo.x, hi.y - lo.y) * 0.5f;
        finite++;
    }

    if (finite == 0) return broad_phase_all_pairs();

    // Cell size from the *average* body, not the largest. Sizing it to the
    // largest lets one wall inflate the grid until it degenerates to all-pairs.
    // Large bodies instead land in many cells, which is bounded work.
    const real cell = std::max(extent_sum / static_cast<real>(finite) * 2.0f, 0.25f);

    std::unordered_map<long long, std::vector<size_t>> grid;
    grid.reserve(finite * 2);

    for (size_t i = 0; i < n; i++)
    {
        Vec2 lo, hi;
        if (!body_bounds(*objects[i], lo, hi)) continue;

        const int x0 = static_cast<int>(std::floor(lo.x / cell));
        const int x1 = static_cast<int>(std::floor(hi.x / cell));
        const int y0 = static_cast<int>(std::floor(lo.y / cell));
        const int y1 = static_cast<int>(std::floor(hi.y / cell));

        for (int gx = x0; gx <= x1; gx++)
            for (int gy = y0; gy <= y1; gy++)
                grid[(static_cast<long long>(gx) << 32) ^ static_cast<unsigned>(gy)].push_back(i);
    }

    std::vector<std::pair<size_t, size_t>> pairs;

    for (const auto& [key, bucket] : grid)
        for (size_t a = 0; a < bucket.size(); a++)
            for (size_t b = a + 1; b < bucket.size(); b++)
                pairs.emplace_back(std::min(bucket[a], bucket[b]),
                                   std::max(bucket[a], bucket[b]));

    // A plane reaches everywhere, so it pairs with every finite body.
    for (const size_t p : planes)
        for (size_t i = 0; i < n; i++)
        {
            Vec2 lo, hi;
            if (!body_bounds(*objects[i], lo, hi)) continue;
            pairs.emplace_back(std::min(p, i), std::max(p, i));
        }

    // A body spanning several cells produces the same pair more than once.
    std::sort(pairs.begin(), pairs.end());
    pairs.erase(std::unique(pairs.begin(), pairs.end()), pairs.end());

    return pairs;
}

std::vector<std::pair<size_t, size_t>> Scene::candidate_pairs() const
{
    return (broad_phase == BroadPhase::SpatialHash) ? broad_phase_hash()
                                                    : broad_phase_all_pairs();
}

// Normalises each pair so the movable shape is A, keeping the manifold normal
// pointing from B toward A for every shape combination.
void Scene::narrow_phase(Object& x, Object& y, std::vector<Manifold>& out)
{
    auto* cx = dynamic_cast<Circle*>(&x);
    auto* cy = dynamic_cast<Circle*>(&y);

    Vec2 norm{0.0f, 0.0f};
    real penetration = 0.0f;

    if (cx && cy)
    {
        if (!circle_vs_circle(*cx, *cy, penetration)) return;

        const Vec2 diff = cx->position - cy->position;
        norm = (diff.length() > 0.0f) ? diff.normalized() : Vec2{1.0f, 0.0f};
        out.push_back(Manifold(cx, cy, true, norm, penetration,
                               cx->position - norm * cx->radius));
        return;
    }

    if (cx || cy)
    {
        Circle* circle = cx ? cx : cy;
        Object& other  = cx ? y : x;

        if (auto* box = dynamic_cast<Box*>(&other))
        {
            if (!box_vs_circle(*box, *circle, norm, penetration)) return;
            out.push_back(Manifold(circle, box, true, norm, penetration,
                                   circle->position - norm * circle->radius));
            return;
        }

        if (auto* plane = dynamic_cast<Plane*>(&other))
        {
            if (!circle_vs_plane(*plane, *circle, norm, penetration)) return;
            out.push_back(Manifold(circle, plane, true, norm, penetration,
                                   circle->position - norm * circle->radius));
        }
        return;
    }

    auto* bx = dynamic_cast<Box*>(&x);
    auto* by = dynamic_cast<Box*>(&y);

    if (bx && !by)
    {
        if (auto* plane = dynamic_cast<Plane*>(&y))
            push_box_plane_contacts(*plane, *bx, out);
        return;
    }

    if (by && !bx)
    {
        if (auto* plane = dynamic_cast<Plane*>(&x))
            push_box_plane_contacts(*plane, *by, out);
        return;
    }

    if (!bx || !by) return;

    Vec2 points[2];
    real depths[2];

    const int count = box_vs_box_contacts(*bx, *by, norm, points, depths);

    for (int i = 0; i < count; i++)
    {
        Manifold m(bx, by, true, norm, depths[i], points[i]);
        m.point_index = i;
        out.push_back(m);
    }
}

// A face contact has two points and each needs its own accumulated impulse,
// otherwise the pair resolves as if pinned at one point and the box tips.
void Scene::push_box_plane_contacts(const Plane& p, Box& b, std::vector<Manifold>& out)
{
    Vec2 norm{0.0f, 0.0f};
    Vec2 points[2];
    real depths[2];

    const int count = box_vs_plane_contacts(p, b, norm, points, depths);

    for (int i = 0; i < count; i++)
    {
        Manifold m(&b, const_cast<Plane*>(&p), true, norm, depths[i], points[i]);
        m.point_index = i;
        out.push_back(m);
    }
}
void Scene::prepare_contact(Manifold& m)
{
    if (m.A->asleep && m.B->asleep)
    {
        contacts_skipped_asleep++;
        return;
    }

    const Vec2 ra = m.contact_point - m.A->position;
    const Vec2 rb = m.contact_point - m.B->position;

    const Vec2 v_ab = (m.A->velocity + ra.perpendicular() * m.A->angular_velocity)
                    - (m.B->velocity + rb.perpendicular() * m.B->angular_velocity);
    const real vel_normal = Vec2::dot(v_ab, m.normal);

    m.bias = (vel_normal < -RESTITUTION_THRESHOLD)
           ? m.A->restitution * m.B->restitution * vel_normal
           : 0.0f;

    const auto cached = contact_cache.find({m.A, m.B, m.point_index});
    m.normal_impulse  = (cached != contact_cache.end()) ? cached->second.normal  : 0.0f;
    m.tangent_impulse = (cached != contact_cache.end()) ? cached->second.tangent : 0.0f;

    if (m.normal_impulse != 0.0f || m.tangent_impulse != 0.0f)
    {
        const Vec2 impulse = m.normal * m.normal_impulse
                           + m.normal.perpendicular() * m.tangent_impulse;

        m.A->velocity += impulse / m.A->mass;
        m.B->velocity -= impulse / m.B->mass;
        m.A->angular_velocity += m.A->inv_inertia * Vec2::cross(ra, impulse);
        m.B->angular_velocity -= m.B->inv_inertia * Vec2::cross(rb, impulse);
    }
}

void Scene::solve_velocity(Manifold& m)
{
    if (!m.colliding) return;
    if (m.A->asleep && m.B->asleep) return;

    Object* A = m.A;
    Object* B = m.B;

    const real inv_mass_a   = 1.0f / A->mass;
    const real inv_mass_b   = 1.0f / B->mass;
    const real inv_mass_sum = inv_mass_a + inv_mass_b;
    if (inv_mass_sum <= 0.0f) return;

    // Offsets from each centre to the contact point. An impulse applied there
    // produces torque as well as force, which is what makes a body roll.
    const Vec2 ra = m.contact_point - A->position;
    const Vec2 rb = m.contact_point - B->position;

    // Velocity of the material point at the contact, not of the centre.
    auto contact_velocity = [&]
    {
        return (A->velocity + ra.perpendicular() * A->angular_velocity)
             - (B->velocity + rb.perpendicular() * B->angular_velocity);
    };

    const real rn_a = Vec2::cross(ra, m.normal);
    const real rn_b = Vec2::cross(rb, m.normal);

    // Effective mass along the normal, including how much of the impulse goes
    // into spin rather than translation.
    const real k_normal = inv_mass_sum
                        + A->inv_inertia * rn_a * rn_a
                        + B->inv_inertia * rn_b * rn_b;
    if (k_normal <= 0.0f) return;

    const real vel_normal = Vec2::dot(contact_velocity(), m.normal);
    const real j = -(vel_normal + m.bias) / k_normal;

    // Clamp the running total, not this pass's delta: a contact may only push.
    const real total = std::max(m.normal_impulse + j, 0.0f);
    const real delta = total - m.normal_impulse;
    m.normal_impulse = total;

    const Vec2 impulse = m.normal * delta;
    A->velocity += impulse * inv_mass_a;
    B->velocity -= impulse * inv_mass_b;
    A->angular_velocity += A->inv_inertia * Vec2::cross(ra, impulse);
    B->angular_velocity -= B->inv_inertia * Vec2::cross(rb, impulse);

    // Friction, against the post-normal-impulse contact velocity. The limit
    // uses the running normal total, so a contact under more load resists more.
    const Vec2 tangent = m.normal.perpendicular();

    const real rt_a = Vec2::cross(ra, tangent);
    const real rt_b = Vec2::cross(rb, tangent);

    const real k_tangent = inv_mass_sum
                         + A->inv_inertia * rt_a * rt_a
                         + B->inv_inertia * rt_b * rt_b;
    if (k_tangent <= 0.0f) return;

    const real vel_tangent = Vec2::dot(contact_velocity(), tangent);
    const real jt          = -vel_tangent / k_tangent;

    const real limit = std::sqrt(A->friction * B->friction) * m.normal_impulse;

    const real total_t = std::clamp(m.tangent_impulse + jt, -limit, limit);
    const real delta_t = total_t - m.tangent_impulse;
    m.tangent_impulse  = total_t;

    const Vec2 friction_impulse = tangent * delta_t;
    A->velocity += friction_impulse * inv_mass_a;
    B->velocity -= friction_impulse * inv_mass_b;
    A->angular_velocity += A->inv_inertia * Vec2::cross(ra, friction_impulse);
    B->angular_velocity -= B->inv_inertia * Vec2::cross(rb, friction_impulse);
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

// The box projects onto the plane normal with radius |h.x*n.x| + |h.y*n.y|,
// which is the support distance toward the plane whatever the normal angle.
// Rotation aware: the projection uses the box's own axes, so it is correct at
// any orientation. Returns up to two contact points, which is what keeps a box
// resting flat on a plane from pivoting about a single point and tipping.
int Scene::box_vs_plane_contacts(const Plane& p, const Box& b, Vec2& norm,
                                 Vec2 points[2], real depths[2]) const
{
    const real projected = project_box(b, p.normal);
    const real distance  = Vec2::dot(b.position - p.position, p.normal);

    if (distance > projected) return 0;

    norm = p.normal;

    const auto corners = b.corners();

    real depth[4];
    real deepest = -std::numeric_limits<real>::max();
    for (int i = 0; i < 4; i++)
    {
        depth[i] = -Vec2::dot(corners[i] - p.position, p.normal);
        deepest  = std::max(deepest, depth[i]);
    }

    // Any corner within a hair of the deepest counts: a flat face gives two.
    const real tolerance = 1e-3f;

    int count = 0;
    for (int i = 0; i < 4 && count < 2; i++)
    {
        if (depth[i] >= 0.0f && depth[i] > deepest - tolerance)
        {
            points[count] = corners[i];
            depths[count] = depth[i];
            count++;
        }
    }
    return count;
}

bool Scene::box_vs_plane(const Plane& p, const Box& b, Vec2& norm, real& penetration) const
{
    Vec2 points[2];
    real depths[2];

    const int count = box_vs_plane_contacts(p, b, norm, points, depths);
    if (count == 0) return false;

    penetration = depths[0];
    for (int i = 1; i < count; i++) penetration = std::max(penetration, depths[i]);
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

// Half-width of a box projected onto an axis.
static real project_box(const Box& box, const Vec2& axis)
{
    const Vec2 half = box.get_half_body();
    return std::fabs(Vec2::dot(box.axis_x() * half.x, axis))
         + std::fabs(Vec2::dot(box.axis_y() * half.y, axis));
}

// The edge of `box` whose outward normal is most aligned with `direction`.
// Corners come back counter-clockwise, so for edge i the outward normal is the
// edge vector rotated clockwise.
static void extreme_edge(const Box& box, const Vec2& direction,
                         Vec2& p0, Vec2& p1, Vec2& out_normal)
{
    const auto c = box.corners();

    real best = -std::numeric_limits<real>::max();
    int  best_i = 0;

    for (int i = 0; i < 4; i++)
    {
        const Vec2 edge = c[(i + 1) % 4] - c[i];
        const Vec2 n{edge.y, -edge.x};
        const real d = Vec2::dot(n.normalized(), direction);

        if (d > best) { best = d; best_i = i; }
    }

    p0 = c[best_i];
    p1 = c[(best_i + 1) % 4];

    const Vec2 edge = p1 - p0;
    out_normal = Vec2{edge.y, -edge.x}.normalized();
}

// Clips a segment to the half-space dot(p, axis) <= limit, keeping order.
static int clip_segment(Vec2 in0, Vec2 in1, const Vec2& axis, real limit, Vec2 out[2])
{
    const real d0 = Vec2::dot(in0, axis) - limit;
    const real d1 = Vec2::dot(in1, axis) - limit;

    int n = 0;
    if (d0 <= 0.0f) out[n++] = in0;
    if (d1 <= 0.0f) out[n++] = in1;

    // One endpoint each side: add the crossing point.
    if (d0 * d1 < 0.0f && n < 2)
    {
        const real t = d0 / (d0 - d1);
        out[n++] = in0 + (in1 - in0) * t;
    }
    return n;
}

// Separating axis test for two oriented boxes, with reference-face clipping to
// produce up to two contact points. One point is not enough: a box resting flat
// would pivot about it and tip for no reason.
int Scene::box_vs_box_contacts(const Box& a, const Box& b, Vec2& norm,
                               Vec2 points[2], real depths[2]) const
{
    const Vec2 axes[4] = {a.axis_x(), a.axis_y(), b.axis_x(), b.axis_y()};
    const Vec2 ab      = a.position - b.position;   // B toward A

    real best_overlap = std::numeric_limits<real>::max();
    int  best_axis    = -1;

    for (int i = 0; i < 4; i++)
    {
        const real overlap = project_box(a, axes[i]) + project_box(b, axes[i])
                           - std::fabs(Vec2::dot(ab, axes[i]));
        if (overlap <= 0.0f) return 0;

        if (overlap < best_overlap) { best_overlap = overlap; best_axis = i; }
    }

    norm = axes[best_axis];
    if (Vec2::dot(ab, norm) < 0.0f) norm = norm * -1.0f;   // B toward A

    // The box owning the separating axis provides the reference face.
    const Box& reference = (best_axis < 2) ? a : b;
    const Box& incident  = (best_axis < 2) ? b : a;
    const Vec2 ref_dir   = (best_axis < 2) ? norm * -1.0f : norm;

    Vec2 r0, r1, ref_normal;
    extreme_edge(reference, ref_dir, r0, r1, ref_normal);

    Vec2 i0, i1, incident_normal;
    extreme_edge(incident, ref_dir * -1.0f, i0, i1, incident_normal);

    // Clip the incident edge to the reference face's side planes.
    const Vec2 side = (r1 - r0).normalized();

    Vec2 clipped[2];
    if (clip_segment(i0, i1, side * -1.0f, -Vec2::dot(r0, side), clipped) < 2) return 0;
    if (clip_segment(clipped[0], clipped[1], side, Vec2::dot(r1, side), clipped) < 2) return 0;

    // Keep only the points behind the reference face.
    int count = 0;
    for (int i = 0; i < 2; i++)
    {
        const real depth = -Vec2::dot(clipped[i] - r0, ref_normal);
        if (depth >= 0.0f)
        {
            points[count] = clipped[i];
            depths[count] = depth;
            count++;
        }
    }
    return count;
}

bool Scene::box_vs_box(const Box& a, const Box& b, Vec2& norm, real& penetration) const
{
    Vec2 points[2];
    real depths[2];

    const int count = box_vs_box_contacts(a, b, norm, points, depths);
    if (count == 0) return false;

    penetration = depths[0];
    for (int i = 1; i < count; i++) penetration = std::max(penetration, depths[i]);
    return true;
}

bool Scene::box_vs_circle(const Box& a, const Circle& c, Vec2& norm, real& penetration) const
{
    // Work in the box's own frame, where it is axis-aligned by construction,
    // then rotate the result back. get_min/get_max are the *bounding* box and
    // would be wrong here for any non-zero orientation.
    const Vec2 ax = a.axis_x();
    const Vec2 ay = a.axis_y();
    const Vec2 half = a.get_half_body();

    const Vec2 offset = c.position - a.position;
    const Vec2 local{Vec2::dot(offset, ax), Vec2::dot(offset, ay)};

    const Vec2 local_closest{std::clamp(local.x, -half.x, half.x),
                             std::clamp(local.y, -half.y, half.y)};

    const Vec2 min = a.position - half;   // retained for the deep-penetration branch
    const Vec2 max = a.position + half;
    const Vec2 closest = a.position + ax * local_closest.x + ay * local_closest.y;
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
