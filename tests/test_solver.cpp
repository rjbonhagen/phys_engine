#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>
#include <utility>

#include "Scene.hpp"
#include "phys/AABB.hpp"
#include "phys/Circle.hpp"
#include "phys/Plane.hpp"

static const phys::Vec2 ZERO{0.0f, 0.0f};
static const phys::Vec2 GRAVITY{0.0f, -9.8f};

static constexpr phys::real DT = 1.0f / 120.0f;

// A floor whose top face sits at y = 2, clear of the circle border clamp at
// y = radius. A floor at y = 0 would have circles resting on the clamp instead
// of on the box, which silences contact entirely.
static void add_floor(Scene& s, phys::real restitution = 0.2f)
{
    s.add_aabb({-2.0f, -2.0f}, {202.0f, 2.0f}, ZERO, ZERO, ZERO, INFINITY, restitution);
}

TEST_CASE("Solver settles a box stack")
{
    // Five unit boxes, so the ideal centres are y = 2.5, 3.5, 4.5, 5.5, 6.5.
    Scene scene{20.0f, 40.0f};
    add_floor(scene);
    for (int i = 0; i < 5; i++)
        scene.add_aabb({9.0f, 2.02f + 1.0f * i}, {11.0f, 3.02f + 1.0f * i},
                       ZERO, ZERO, GRAVITY, 1.0f, 0.1f);

    auto& o = scene.get_objects();

    for (int i = 0; i < static_cast<int>(30.0f / DT); i++) scene.step(DT);

    SECTION("every box ends within tolerance of its resting height")
    {
        for (int i = 0; i < 5; i++)
        {
            const phys::real ideal = 2.5f + 1.0f * i;
            const phys::real sink  = ideal - o[i + 1]->position.y;

            REQUIRE(sink >= -0.01f);   // must not float above the ideal
            REQUIRE(sink < 0.12f);     // measured worst case is about 0.07
        }
    }

    SECTION("the stack is at rest, not jittering")
    {
        for (int i = 0; i < 5; i++)
            REQUIRE(o[i + 1]->velocity.length() < 0.05f);
    }

    SECTION("boxes stay stacked rather than drifting apart")
    {
        for (int i = 0; i < 4; i++)
        {
            const phys::real gap = o[i + 2]->position.y - o[i + 1]->position.y;
            REQUIRE(gap == Catch::Approx(1.0f).margin(0.05f));
        }
    }

    SECTION("nothing goes non-finite")
    {
        for (int i = 0; i < 5; i++)
        {
            REQUIRE(std::isfinite(o[i + 1]->position.y));
            REQUIRE(std::isfinite(o[i + 1]->velocity.length()));
        }
    }
}

TEST_CASE("Solver iteration count improves convergence")
{
    // Ten boxes overlapping at t = 0, so every contact exists on the first
    // step and the stack is still converging when sampled.
    auto residual_speed = [](int iterations)
    {
        Scene scene{20.0f, 60.0f};
        scene.set_solver_iterations(iterations);
        add_floor(scene);
        for (int i = 0; i < 10; i++)
            scene.add_aabb({9.0f, 2.0f + 0.98f * i}, {11.0f, 3.0f + 0.98f * i},
                           ZERO, ZERO, GRAVITY, 1.0f, 0.1f);

        auto& o = scene.get_objects();
        for (int i = 0; i < static_cast<int>(0.25f / DT); i++) scene.step(DT);

        phys::real worst = 0.0f;
        for (int i = 0; i < 10; i++) worst = std::max(worst, o[i + 1]->velocity.length());
        return worst;
    };

    REQUIRE(residual_speed(8) < residual_speed(1));
}

// Ramp inclined by `degrees`, rising to the right, surface through the origin.
// Outward normal is (-sin, cos). Returns the distance travelled along the ramp
// (positive uphill) and the final speed.
static std::pair<phys::real, phys::real> ramp_run(phys::real degrees, phys::real mu,
                                                  phys::real speed_uphill, phys::real seconds)
{
    const phys::real theta = degrees * 3.14159265f / 180.0f;
    const phys::Vec2 normal{-std::sin(theta), std::cos(theta)};
    const phys::Vec2 uphill{ std::cos(theta), std::sin(theta)};

    Scene scene{60.0f, 60.0f};
    scene.add_plane({0.0f, 0.0f}, normal, 0.1f);

    const phys::Vec2 surface = uphill * 20.0f;
    scene.add_circle(surface + normal * 0.5f, uphill * speed_uphill, ZERO, 0.5f, GRAVITY, 1.0f, 0.1f);

    auto& o = scene.get_objects();
    o[0]->friction = mu;
    o[1]->friction = mu;

    const phys::Vec2 start = o[1]->position;
    for (int i = 0; i < static_cast<int>(seconds / DT); i++) scene.step(DT);

    return {phys::Vec2::dot(o[1]->position - start, uphill), o[1]->velocity.length()};
}

// Builds the same deterministic scene twice so two broad phases can be compared.
static void populate_crowd(Scene& scene, unsigned seed, int bodies)
{
    scene.create_walls(1.0f);

    unsigned state = seed;
    auto rnd = [&state]
    {
        state = state * 1664525u + 1013904223u;
        return static_cast<phys::real>((state >> 8) & 0xFFFF) / 65535.0f;
    };

    for (int i = 0; i < bodies; i++)
        scene.add_circle({1.0f + rnd() * 18.0f, 1.0f + rnd() * 18.0f},
                         {rnd() * 6.0f - 3.0f, rnd() * 6.0f - 3.0f},
                         ZERO, 0.3f, GRAVITY, 1.0f, 0.4f);
}

TEST_CASE("Continuous detection stops fast bodies")
{
    // Thin static wall spanning x in [10, 10.2]. Measured without CCD and
    // without the old speed clamp, a circle at 1000 or 5000 units/s passed
    // clean through and reached the far scene bound at x = 199.8.
    auto farthest_x = [](phys::real speed)
    {
        Scene scene{200.0f, 40.0f};
        scene.add_aabb({10.0f, 0.0f}, {10.2f, 40.0f}, ZERO, ZERO, ZERO, INFINITY, 0.5f);
        scene.add_circle({5.0f, 20.0f}, {speed, 0.0f}, ZERO, 0.2f, ZERO, 1.0f, 0.5f);

        auto* c = scene.get_objects()[1].get();
        phys::real max_x = c->position.x;

        for (int i = 0; i < static_cast<int>(3.0f / DT); i++)
        {
            scene.step(DT);
            max_x = std::max(max_x, c->position.x);
        }
        return max_x;
    };

    SECTION("a bullet does not pass through a thin wall")
    {
        // Far face plus one radius is 10.4; anything beyond means it tunnelled.
        REQUIRE(farthest_x(1000.0f) < 10.4f);
        REQUIRE(farthest_x(5000.0f) < 10.4f);
    }

    SECTION("slow bodies are unaffected and behave as before")
    {
        REQUIRE(farthest_x(20.0f) < 10.4f);
        REQUIRE(farthest_x(60.0f) < 10.4f);
    }

    SECTION("the sweep is a no-op for bodies moving under one radius")
    {
        Scene scene{20.0f, 40.0f};
        scene.add_aabb({0.0f, 0.0f}, {20.0f, 2.0f}, ZERO, ZERO, ZERO, INFINITY, 0.2f);
        scene.add_circle({10.0f, 8.0f}, ZERO, ZERO, 0.5f, GRAVITY, 1.0f, 0.3f);

        for (int i = 0; i < static_cast<int>(5.0f / DT); i++)
        {
            scene.step(DT);
            // Gravity alone never moves a 0.5-radius body a radius per step.
            REQUIRE(scene.get_stats().toi_clamps == 0);
        }
    }

    SECTION("the sweep reports a clamp when it actually fires")
    {
        Scene scene{200.0f, 40.0f};
        scene.add_aabb({10.0f, 0.0f}, {10.2f, 40.0f}, ZERO, ZERO, ZERO, INFINITY, 0.5f);
        scene.add_circle({5.0f, 20.0f}, {1000.0f, 0.0f}, ZERO, 0.2f, ZERO, 1.0f, 0.5f);

        size_t clamps = 0;
        for (int i = 0; i < static_cast<int>(1.0f / DT); i++)
        {
            scene.step(DT);
            clamps += scene.get_stats().toi_clamps;
        }
        REQUIRE(clamps > 0);
    }

    SECTION("a bullet is stopped by a plane too")
    {
        Scene scene{200.0f, 40.0f};
        scene.add_plane({10.0f, 0.0f}, {-1.0f, 0.0f}, 0.5f);   // solid to the right
        scene.add_circle({5.0f, 20.0f}, {2000.0f, 0.0f}, ZERO, 0.2f, ZERO, 1.0f, 0.5f);

        auto* c = scene.get_objects()[1].get();
        for (int i = 0; i < static_cast<int>(2.0f / DT); i++) scene.step(DT);

        REQUIRE(c->position.x < 10.0f);
    }
}

TEST_CASE("A box rests on and slides along a plane")
{
    SECTION("a box released above a plane comes to rest on it")
    {
        // Regression: this pair was never implemented, so a box fell straight
        // through. From y = 15 it reached y = -63 instead of resting at 5.5.
        Scene scene{20.0f, 40.0f};
        scene.add_plane({0.0f, 5.0f}, {0.0f, 1.0f}, 0.2f);
        scene.add_aabb({9.0f, 15.0f}, {11.0f, 16.0f}, ZERO, ZERO, GRAVITY, 1.0f, 0.2f);

        auto& o = scene.get_objects();
        for (int i = 0; i < static_cast<int>(6.0f / DT); i++) scene.step(DT);

        REQUIRE(o[1]->position.y == Catch::Approx(5.5f).margin(0.02f));
        REQUIRE(o[1]->velocity.length() == Catch::Approx(0.0f).margin(0.01f));
        REQUIRE(scene.get_contacts().size() == 1);
    }

    SECTION("friction holds a box on a shallow ramp and not on a steep one")
    {
        auto slide = [](phys::real degrees, phys::real mu)
        {
            const phys::real theta = degrees * 3.14159265f / 180.0f;
            const phys::Vec2 normal{-std::sin(theta), std::cos(theta)};
            const phys::Vec2 uphill{ std::cos(theta), std::sin(theta)};

            Scene scene{80.0f, 80.0f};
            scene.add_plane({0.0f, 0.0f}, normal, 0.1f);

            const phys::real half = 0.5f;
            const phys::real projected = std::fabs(half * normal.x) + std::fabs(half * normal.y);
            const phys::Vec2 centre = uphill * 25.0f + normal * projected;

            scene.add_aabb(centre - phys::Vec2{half, half}, centre + phys::Vec2{half, half},
                           ZERO, ZERO, GRAVITY, 1.0f, 0.1f);

            auto& o = scene.get_objects();
            o[0]->friction = mu;
            o[1]->friction = mu;

            const phys::Vec2 start = o[1]->position;
            for (int i = 0; i < static_cast<int>(12.0f / DT); i++) scene.step(DT);

            return phys::Vec2::dot(o[1]->position - start, uphill);
        };

        // tan(30) = 0.577, tan(20) = 0.364.
        REQUIRE(std::fabs(slide(30.0f, 0.8f)) < 0.2f);
        REQUIRE(slide(30.0f, 0.2f) < -20.0f);
        REQUIRE(std::fabs(slide(20.0f, 0.5f)) < 0.2f);
        REQUIRE(slide(20.0f, 0.1f) < -20.0f);
    }

    SECTION("a box clear of the plane reports no contact")
    {
        Scene scene{20.0f, 40.0f};
        scene.add_plane({0.0f, 5.0f}, {0.0f, 1.0f}, 0.2f);
        scene.add_aabb({9.0f, 20.0f}, {11.0f, 21.0f}, ZERO, ZERO, ZERO, 1.0f, 0.2f);

        scene.step(DT);
        REQUIRE(scene.get_contacts().empty());
    }
}

TEST_CASE("Spatial hash broad phase agrees with all-pairs")
{
    SECTION("it finds the same contacts over a long run")
    {
        // The strong property: if the grid ever misses a pair, the contact
        // count diverges from the reference on the step it was missed.
        Scene reference{20.0f, 20.0f};
        Scene hashed{20.0f, 20.0f};
        populate_crowd(reference, 99u, 120);
        populate_crowd(hashed, 99u, 120);

        reference.set_broad_phase(Scene::BroadPhase::AllPairs);
        hashed.set_broad_phase(Scene::BroadPhase::SpatialHash);

        for (int i = 0; i < 600; i++)
        {
            reference.step(DT);
            hashed.step(DT);

            REQUIRE(hashed.get_stats().contacts == reference.get_stats().contacts);
        }
    }

    SECTION("it tests far fewer pairs than all-pairs")
    {
        Scene scene{20.0f, 20.0f};
        populate_crowd(scene, 7u, 200);
        for (int i = 0; i < 60; i++) scene.step(DT);

        scene.set_broad_phase(Scene::BroadPhase::AllPairs);
        const size_t all = scene.candidate_pairs().size();

        scene.set_broad_phase(Scene::BroadPhase::SpatialHash);
        const size_t hashed = scene.candidate_pairs().size();

        REQUIRE(hashed < all / 4);
    }

    SECTION("planes are still paired, since they cannot be bucketed")
    {
        Scene scene{20.0f, 40.0f};
        scene.set_broad_phase(Scene::BroadPhase::SpatialHash);
        scene.add_plane({0.0f, 5.0f}, {0.0f, 1.0f}, 0.2f);
        scene.add_circle({10.0f, 15.0f}, ZERO, ZERO, 0.5f, GRAVITY, 1.0f, 0.2f);

        for (int i = 0; i < static_cast<int>(5.0f / DT); i++) scene.step(DT);

        // Would fall straight through if the plane were dropped from the grid.
        REQUIRE(scene.get_objects()[1]->position.y == Catch::Approx(5.5f).margin(0.02f));
    }

    SECTION("a scene of only planes falls back to all-pairs")
    {
        Scene scene{20.0f, 20.0f};
        scene.set_broad_phase(Scene::BroadPhase::SpatialHash);
        scene.add_plane({0.0f, 0.0f}, {0.0f, 1.0f}, 0.2f);
        scene.add_plane({0.0f, 10.0f}, {0.0f, -1.0f}, 0.2f);

        scene.step(DT);   // must not divide by zero on an empty grid
        REQUIRE(scene.get_contacts().empty());
    }
}

TEST_CASE("Scene reports contacts for debug rendering")
{
    SECTION("a resting circle on a plane reports one contact at the surface")
    {
        Scene scene{20.0f, 40.0f};
        scene.add_plane({0.0f, 5.0f}, {0.0f, 1.0f}, 0.2f);
        scene.add_circle({10.0f, 15.0f}, ZERO, ZERO, 0.5f, GRAVITY, 1.0f, 0.2f);

        for (int i = 0; i < static_cast<int>(5.0f / DT); i++) scene.step(DT);

        const auto& contacts = scene.get_contacts();
        REQUIRE(contacts.size() == 1);

        // On the circle surface facing the plane, so just above y = 5.
        REQUIRE(contacts[0].point.y == Catch::Approx(5.0f).margin(0.05f));
        REQUIRE(contacts[0].point.x == Catch::Approx(10.0f).margin(0.05f));
        REQUIRE(contacts[0].normal.y == Catch::Approx(1.0f).margin(0.01f));
        REQUIRE(contacts[0].penetration >= 0.0f);
    }

    SECTION("box contacts are reported at the overlap centre, not the origin")
    {
        Scene scene{40.0f, 40.0f};
        scene.add_aabb({8.0f, 10.0f}, {12.0f, 20.0f}, ZERO, ZERO, ZERO, INFINITY, 0.2f);
        scene.add_aabb({11.0f, 12.0f}, {15.0f, 18.0f}, ZERO, ZERO, ZERO, INFINITY, 0.2f);

        scene.step(DT);

        const auto& contacts = scene.get_contacts();
        REQUIRE(contacts.size() == 1);

        // Overlap spans x 11..12, y 12..18, so the centre is (11.5, 15).
        REQUIRE(contacts[0].point.x == Catch::Approx(11.5f));
        REQUIRE(contacts[0].point.y == Catch::Approx(15.0f));
    }

    SECTION("no contacts are reported when nothing touches")
    {
        Scene scene{40.0f, 40.0f};
        scene.add_aabb({2.0f, 2.0f}, {4.0f, 4.0f}, ZERO, ZERO, ZERO, INFINITY, 0.2f);
        scene.add_aabb({20.0f, 20.0f}, {22.0f, 22.0f}, ZERO, ZERO, ZERO, INFINITY, 0.2f);

        scene.step(DT);

        REQUIRE(scene.get_contacts().empty());
    }
}

TEST_CASE("A circle comes to rest on a plane")
{
    // Horizontal plane through y = 5, solid below. Independent of the ramp
    // cases: this checks the plane collider supports a body at all.
    Scene scene{20.0f, 40.0f};
    scene.add_plane({0.0f, 5.0f}, {0.0f, 1.0f}, 0.2f);
    scene.add_circle({10.0f, 15.0f}, ZERO, ZERO, 0.5f, GRAVITY, 1.0f, 0.2f);

    auto& o = scene.get_objects();
    for (int i = 0; i < static_cast<int>(10.0f / DT); i++) scene.step(DT);

    SECTION("it rests one radius above the surface")
    {
        // 5.5 ideal, less the 0.01 penetration slop the solver leaves.
        REQUIRE(o[1]->position.y == Catch::Approx(5.5f).margin(0.02f));
    }

    SECTION("it is at rest and did not fall through")
    {
        REQUIRE(o[1]->velocity.length() == Catch::Approx(0.0f).margin(0.01f));
        REQUIRE(o[1]->position.y > 5.0f);
    }

    SECTION("the plane does not move")
    {
        REQUIRE(o[0]->position == phys::Vec2{0.0f, 5.0f});
        REQUIRE(o[0]->velocity == ZERO);
    }

    SECTION("a plane needs a non-zero normal")
    {
        REQUIRE_THROWS_AS((phys::Plane{{0.0f, 0.0f}, {0.0f, 0.0f}}), std::invalid_argument);
    }
}

TEST_CASE("Friction stops a circle on a ramp")
{
    // tan(30 deg) = 0.577, so friction holds the circle when mu exceeds that
    // and lets it slide when it does not.

    SECTION("a circle launched up a ramp comes to rest instead of sliding on")
    {
        const auto [travelled, speed] = ramp_run(30.0f, 0.8f, 6.0f, 15.0f);

        REQUIRE(speed == Catch::Approx(0.0f).margin(0.01f));
        REQUIRE(travelled > 1.0f);    // it did climb
        REQUIRE(travelled < 2.0f);    // and friction stopped it rather than letting it run
    }

    SECTION("uphill stopping distance matches v^2 / (2 g (sin + mu cos))")
    {
        const phys::real theta = 30.0f * 3.14159265f / 180.0f;
        const phys::real decel = 9.8f * (std::sin(theta) + 0.8f * std::cos(theta));
        const phys::real predicted = (6.0f * 6.0f) / (2.0f * decel);

        const auto [travelled, speed] = ramp_run(30.0f, 0.8f, 6.0f, 15.0f);

        REQUIRE(travelled == Catch::Approx(predicted).margin(0.1f));
    }

    SECTION("friction above the slope angle holds a resting circle in place")
    {
        const auto [travelled, speed] = ramp_run(30.0f, 0.8f, 0.0f, 15.0f);

        REQUIRE(std::fabs(travelled) < 0.1f);
        REQUIRE(speed == Catch::Approx(0.0f).margin(0.01f));
    }

    SECTION("friction below the slope angle lets it slide down")
    {
        const auto [travelled, speed] = ramp_run(30.0f, 0.2f, 0.0f, 15.0f);

        REQUIRE(travelled < -5.0f);   // slid a long way downhill
    }

    SECTION("a shallower ramp needs less friction to hold")
    {
        // tan(20 deg) = 0.364, so mu = 0.5 holds here but would not at 30 deg.
        const auto [held, held_speed]   = ramp_run(20.0f, 0.5f, 0.0f, 15.0f);
        const auto [slid, slid_speed]   = ramp_run(20.0f, 0.1f, 0.0f, 15.0f);

        REQUIRE(std::fabs(held) < 0.1f);
        REQUIRE(slid < -5.0f);
    }
}

TEST_CASE("Friction brings a sliding body to rest")
{
    // Flat-floor companion to the ramp cases above: same tangent-impulse path,
    // but with an analytic result that does not involve the slope angle.
    auto slide_distance = [](phys::real mu, bool use_circle)
    {
        Scene scene{200.0f, 40.0f};
        add_floor(scene, 0.1f);

        if (use_circle) scene.add_circle({5.0f, 2.5f}, {12.0f, 0.0f}, ZERO, 0.5f, GRAVITY, 1.0f, 0.1f);
        else            scene.add_aabb({5.0f, 2.0f}, {6.0f, 3.0f}, {12.0f, 0.0f}, ZERO, GRAVITY, 1.0f, 0.1f);

        auto& o = scene.get_objects();
        o[0]->friction = mu;
        o[1]->friction = mu;

        const phys::real start = o[1]->position.x;
        for (int i = 0; i < static_cast<int>(20.0f / DT); i++) scene.step(DT);

        return std::pair{o[1]->position.x - start, o[1]->velocity.x};
    };

    SECTION("a box stops, and sooner with more friction")
    {
        const auto [low, v_low]   = slide_distance(0.3f, false);
        const auto [high, v_high] = slide_distance(0.6f, false);

        REQUIRE(v_low == Catch::Approx(0.0f).margin(0.01f));
        REQUIRE(v_high == Catch::Approx(0.0f).margin(0.01f));
        REQUIRE(high < low);
    }

    SECTION("stopping distance matches v^2 / (2 mu g)")
    {
        const auto [measured, v] = slide_distance(0.3f, false);
        const phys::real predicted = (12.0f * 12.0f) / (2.0f * 0.3f * 9.8f);

        REQUIRE(measured == Catch::Approx(predicted).margin(0.5f));
    }

    SECTION("a circle behaves the same as a box")
    {
        const auto [box, v_box]       = slide_distance(0.3f, false);
        const auto [circle, v_circle] = slide_distance(0.3f, true);

        REQUIRE(circle == Catch::Approx(box).margin(0.5f));
    }

    SECTION("zero friction does not decelerate")
    {
        const auto [distance, v] = slide_distance(0.0f, false);
        REQUIRE(v == Catch::Approx(12.0f).margin(0.01f));
    }
}

TEST_CASE("Restitution suppression lets an elastic body settle")
{
    // At e = 0.9 a resting contact would otherwise bounce back its own gravity
    // increment every step and never come to rest.
    Scene scene{20.0f, 40.0f};
    add_floor(scene, 0.9f);
    scene.add_aabb({9.0f, 6.0f}, {11.0f, 7.0f}, ZERO, ZERO, GRAVITY, 1.0f, 0.9f);

    auto& o = scene.get_objects();
    for (int i = 0; i < static_cast<int>(10.0f / DT); i++) scene.step(DT);

    REQUIRE(o[1]->velocity.length() == Catch::Approx(0.0f).margin(0.001f));
    REQUIRE(o[1]->position.y == Catch::Approx(2.5f).margin(0.02f));
}

TEST_CASE("Settled islands sleep and are skipped by the solver")
{
    Scene scene{20.0f, 40.0f};
    add_floor(scene);
    for (int i = 0; i < 5; i++)
        scene.add_aabb({9.0f, 2.02f + 1.0f * i}, {11.0f, 3.02f + 1.0f * i},
                       ZERO, ZERO, GRAVITY, 1.0f, 0.1f);

    SECTION("a stack sleeps, and every one of its contacts is skipped")
    {
        for (int i = 0; i < static_cast<int>(3.0f / DT); i++) scene.step(DT);

        REQUIRE(scene.count_sleeping() == 5);
        // Five box-box and box-floor contacts in a five-box stack.
        REQUIRE(scene.get_contacts_skipped_asleep() == 5);
    }

    SECTION("an impact wakes the island, which then re-sleeps")
    {
        for (int i = 0; i < static_cast<int>(3.0f / DT); i++) scene.step(DT);
        REQUIRE(scene.count_sleeping() == 5);

        scene.add_aabb({9.0f, 12.0f}, {11.0f, 13.0f}, {0.0f, -8.0f}, ZERO, GRAVITY, 1.0f, 0.1f);

        bool woke = false;
        for (int i = 0; i < static_cast<int>(1.0f / DT); i++)
        {
            scene.step(DT);
            if (scene.count_sleeping() == 0) woke = true;
        }
        REQUIRE(woke);

        for (int i = 0; i < static_cast<int>(4.0f / DT); i++) scene.step(DT);

        REQUIRE(scene.count_sleeping() == 6);
        REQUIRE(scene.get_contacts_skipped_asleep() == 6);
    }

    SECTION("a body in free fall does not sleep")
    {
        Scene air{20.0f, 400.0f};
        air.add_circle({10.0f, 380.0f}, ZERO, ZERO, 0.5f, GRAVITY, 1.0f, 0.5f);

        for (int i = 0; i < static_cast<int>(3.0f / DT); i++) air.step(DT);

        REQUIRE(air.count_sleeping() == 0);
        REQUIRE(air.get_objects()[0]->velocity.length() > 1.0f);
    }
}

TEST_CASE("Scene exposes a configurable iteration count")
{
    Scene scene{10.0f, 10.0f};

    REQUIRE(scene.get_solver_iterations() == 8);

    scene.set_solver_iterations(4);
    REQUIRE(scene.get_solver_iterations() == 4);

    // A non-positive count would skip the solver entirely.
    scene.set_solver_iterations(0);
    REQUIRE(scene.get_solver_iterations() == 1);
    scene.set_solver_iterations(-3);
    REQUIRE(scene.get_solver_iterations() == 1);
}
