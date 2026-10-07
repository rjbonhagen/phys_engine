#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>

#include "Scene.hpp"
#include "phys/AABB.hpp"
#include "phys/Circle.hpp"

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

TEST_CASE("Friction brings a sliding body to rest")
{
    // A ramp would need an oriented surface, which an AABB cannot represent,
    // and rolling needs angular dynamics the engine does not have yet. So
    // friction is measured as deceleration along a flat floor, which is the
    // same tangent-impulse path a ramp would exercise.
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
