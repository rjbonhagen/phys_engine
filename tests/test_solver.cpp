#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>
#include <utility>

#include "Scene.hpp"
#include "phys/Box.hpp"
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
    s.add_box({-2.0f, -2.0f}, {202.0f, 2.0f}, ZERO, ZERO, ZERO, INFINITY, restitution);
}

TEST_CASE("Solver settles a box stack")
{
    // Five unit boxes, so the ideal centres are y = 2.5, 3.5, 4.5, 5.5, 6.5.
    Scene scene{20.0f, 40.0f};
    add_floor(scene);
    for (int i = 0; i < 5; i++)
        scene.add_box({9.0f, 2.02f + 1.0f * i}, {11.0f, 3.02f + 1.0f * i},
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
            scene.add_box({9.0f, 2.0f + 0.98f * i}, {11.0f, 3.0f + 0.98f * i},
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
        scene.add_box({10.0f, 0.0f}, {10.2f, 40.0f}, ZERO, ZERO, ZERO, INFINITY, 0.5f);
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
        scene.add_box({0.0f, 0.0f}, {20.0f, 2.0f}, ZERO, ZERO, ZERO, INFINITY, 0.2f);
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
        scene.add_box({10.0f, 0.0f}, {10.2f, 40.0f}, ZERO, ZERO, ZERO, INFINITY, 0.5f);
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
        scene.add_box({9.0f, 15.0f}, {11.0f, 16.0f}, ZERO, ZERO, GRAVITY, 1.0f, 0.2f);

        auto& o = scene.get_objects();
        for (int i = 0; i < static_cast<int>(6.0f / DT); i++) scene.step(DT);

        REQUIRE(o[1]->position.y == Catch::Approx(5.5f).margin(0.02f));
        REQUIRE(o[1]->velocity.length() == Catch::Approx(0.0f).margin(0.01f));
        // A flat face against a plane yields two contact points, which is what
        // stops the box pivoting about a single one.
        REQUIRE(scene.get_contacts().size() == 2);
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
            const phys::Vec2 centre = uphill * 25.0f + normal * half;

            scene.add_box(centre - phys::Vec2{half, half}, centre + phys::Vec2{half, half},
                           ZERO, ZERO, GRAVITY, 1.0f, 0.1f);

            auto& o = scene.get_objects();

            // Lie the box flat on the slope. Boxes rotate now, so leaving it
            // axis-aligned would balance it on one corner, which is an unstable
            // equilibrium and tumbles rather than testing friction.
            o[1]->orientation = theta;
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
        scene.add_box({9.0f, 20.0f}, {11.0f, 21.0f}, ZERO, ZERO, ZERO, 1.0f, 0.2f);

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

    SECTION("box contacts are reported on the contact face, not at the origin")
    {
        // Face clipping replaced the single overlap-centre point this used to
        // assert. The overlap spans x 11..12, y 12..18, and the separating axis
        // is x, so both points sit on the shared vertical face.
        Scene scene{40.0f, 40.0f};
        scene.add_box({8.0f, 10.0f}, {12.0f, 20.0f}, ZERO, ZERO, ZERO, INFINITY, 0.2f);
        scene.add_box({11.0f, 12.0f}, {15.0f, 18.0f}, ZERO, ZERO, ZERO, INFINITY, 0.2f);

        scene.step(DT);

        const auto& contacts = scene.get_contacts();
        REQUIRE(contacts.size() == 2);

        for (const auto& c : contacts)
        {
            REQUIRE(c.point.x == Catch::Approx(11.0f).margin(1.0f));
            REQUIRE(c.point.y == Catch::Approx(15.0f).margin(3.001f));
            REQUIRE(c.point.x != 0.0f);
        }
        // Two distinct points, not the same one twice.
        REQUIRE(contacts[0].point.y != contacts[1].point.y);
    }

    SECTION("no contacts are reported when nothing touches")
    {
        Scene scene{40.0f, 40.0f};
        scene.add_box({2.0f, 2.0f}, {4.0f, 4.0f}, ZERO, ZERO, ZERO, INFINITY, 0.2f);
        scene.add_box({20.0f, 20.0f}, {22.0f, 22.0f}, ZERO, ZERO, ZERO, INFINITY, 0.2f);

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

TEST_CASE("Distance joints")
{
    SECTION("a world-anchored joint holds its length")
    {
        Scene scene{40.0f, 40.0f};
        scene.set_solver_iterations(20);
        scene.add_circle({22.0f, 20.0f}, ZERO, ZERO, 0.1f, GRAVITY, 1.0f, 0.0f);
        scene.add_joint(0, {22.0f, 20.0f}, {20.0f, 20.0f});

        auto* c = scene.get_objects()[0].get();
        for (int i = 0; i < static_cast<int>(20.0f / DT); i++) scene.step(DT);

        const phys::real rod = (c->position - phys::Vec2{20.0f, 20.0f}).length();
        REQUIRE(rod == Catch::Approx(2.0f).margin(0.01f));
    }

    SECTION("pendulum period matches the large-amplitude prediction")
    {
        // Released horizontally, so this is a 90 degree amplitude, not the
        // small-angle case. The exact period is T0 * (2/pi) * K(sin(45 deg)),
        // which is 1.1803 times the small-angle T0 = 2 pi sqrt(L/g).
        const phys::real L = 2.0f;
        const phys::Vec2 pivot{20.0f, 20.0f};

        Scene scene{40.0f, 40.0f};
        scene.set_solver_iterations(20);
        scene.add_circle({pivot.x + L, pivot.y}, ZERO, ZERO, 0.1f, GRAVITY, 1.0f, 0.0f);
        scene.add_joint(0, {pivot.x + L, pivot.y}, pivot);

        auto* c = scene.get_objects()[0].get();

        // Consecutive downward crossings of the vertical are one period apart.
        phys::real previous = c->position.x - pivot.x;
        int crossings = 0;
        phys::real first = 0.0f, last = 0.0f;

        for (int i = 1; i <= static_cast<int>(20.0f / DT); i++)
        {
            scene.step(DT);
            const phys::real x = c->position.x - pivot.x;

            if (previous > 0.0f && x <= 0.0f)
            {
                if (crossings == 0) first = i * DT;
                last = i * DT;
                crossings++;
            }
            previous = x;
        }

        REQUIRE(crossings >= 3);

        const phys::real measured  = (last - first) / (crossings - 1);
        const phys::real small     = 6.2831853f * std::sqrt(L / 9.8f);
        const phys::real predicted = 1.1803f * small;

        REQUIRE(measured == Catch::Approx(predicted).margin(0.05f));
    }

    SECTION("a hanging chain holds every link spacing")
    {
        Scene scene{40.0f, 40.0f};
        scene.set_solver_iterations(20);

        for (int i = 0; i < 5; i++)
            scene.add_circle({20.0f, 30.0f - 1.0f * (i + 1)}, ZERO, ZERO, 0.15f,
                             GRAVITY, 1.0f, 0.0f);

        scene.add_joint(0, {20.0f, 29.0f}, {20.0f, 30.0f});
        for (int i = 0; i < 4; i++)
            scene.add_joint(i, i + 1, {20.0f, 29.0f - 1.0f * i}, {20.0f, 28.0f - 1.0f * i});

        auto& o = scene.get_objects();
        for (int i = 0; i < static_cast<int>(10.0f / DT); i++) scene.step(DT);

        for (int i = 0; i < 4; i++)
        {
            const phys::real gap = o[i]->position.y - o[i + 1]->position.y;
            REQUIRE(gap == Catch::Approx(1.0f).margin(0.01f));
        }
        // It hangs straight down rather than drifting sideways.
        REQUIRE(o[4]->position.x == Catch::Approx(20.0f).margin(0.05f));
    }

    SECTION("a joint between two bodies pulls them together")
    {
        Scene scene{60.0f, 60.0f};
        scene.set_solver_iterations(20);
        scene.add_circle({25.0f, 30.0f}, ZERO, ZERO, 0.3f, ZERO, 1.0f, 0.0f);
        scene.add_circle({35.0f, 30.0f}, ZERO, ZERO, 0.3f, ZERO, 1.0f, 0.0f);

        // Anchored at both centres but asked for 4 units while they start 10
        // apart, so the joint has to pull them in.
        scene.add_joint(0, 1, {25.0f, 30.0f}, {35.0f, 30.0f}, 4.0f);

        auto& o = scene.get_objects();
        for (int i = 0; i < static_cast<int>(5.0f / DT); i++) scene.step(DT);

        const phys::real separation = (o[0]->position - o[1]->position).length();
        REQUIRE(separation == Catch::Approx(4.0f).margin(0.1f));
    }

    SECTION("removing a body drops the joints that referenced it")
    {
        Scene scene{40.0f, 40.0f};
        scene.add_circle({22.0f, 20.0f}, ZERO, ZERO, 0.1f, GRAVITY, 1.0f, 0.0f);
        scene.add_joint(0, {22.0f, 20.0f}, {20.0f, 20.0f});

        REQUIRE(scene.get_joints().size() == 1);

        scene.remove_object(0);
        REQUIRE(scene.get_joints().empty());

        scene.step(DT);   // must not dereference the removed body
    }

    SECTION("a degenerate joint request is ignored")
    {
        Scene scene{40.0f, 40.0f};
        scene.add_circle({20.0f, 20.0f}, ZERO, ZERO, 0.1f, ZERO, 1.0f, 0.0f);

        scene.add_joint(0, 0, {20.0f, 20.0f}, {21.0f, 20.0f});   // a body to itself
        scene.add_joint(0, 99, {20.0f, 20.0f}, {21.0f, 20.0f});  // out of range
        scene.add_joint(0, {20.0f, 20.0f}, {21.0f, 20.0f}, -1.0f);  // negative length

        REQUIRE(scene.get_joints().empty());
    }
}

TEST_CASE("Moving a body by hand does not leave things floating")
{
    // A sleeping body skips integration, so whatever was resting on a support
    // hangs in the air when that support is moved out from under it. Scene::wake
    // is what the sandbox calls when dragging.
    auto settle_then_move = [](bool call_wake)
    {
        Scene scene{40.0f, 40.0f};
        scene.add_box({0.0f, 0.0f}, {40.0f, 2.0f}, ZERO, ZERO, ZERO, INFINITY, 0.1f);
        scene.add_box({18.0f, 2.0f}, {22.0f, 3.0f}, ZERO, ZERO, GRAVITY, 1.0f, 0.1f);
        for (int i = 0; i < 3; i++)
            scene.add_box({19.0f, 3.02f + 1.0f * i}, {21.0f, 4.02f + 1.0f * i},
                          ZERO, ZERO, GRAVITY, 1.0f, 0.1f);

        auto& o = scene.get_objects();
        for (int i = 0; i < static_cast<int>(4.0f / DT); i++) scene.step(DT);

        const int slept = scene.count_sleeping();
        const phys::real before = o[4]->position.y;

        // Yank the support aside, the way a drag would.
        if (call_wake) scene.wake(1);
        o[1]->position = phys::Vec2{35.0f, 2.5f};
        if (call_wake) scene.wake(1);

        for (int i = 0; i < static_cast<int>(3.0f / DT); i++) scene.step(DT);

        struct R { int slept; phys::real before; phys::real after; };
        return R{slept, before, o[4]->position.y};
    };

    SECTION("without waking, the stack hangs exactly where it was")
    {
        const auto r = settle_then_move(false);

        REQUIRE(r.slept == 4);                                    // all asleep first
        REQUIRE(r.after == Catch::Approx(r.before).margin(1e-4f)); // never moved
    }

    SECTION("waking drops the stack by the height of the removed support")
    {
        const auto r = settle_then_move(true);

        REQUIRE(r.slept == 4);
        // The support was one unit tall, so the stack lands one unit lower.
        REQUIRE(r.after == Catch::Approx(r.before - 1.0f).margin(0.05f));
    }

    SECTION("removing a body wakes what was resting on it")
    {
        Scene scene{40.0f, 40.0f};
        scene.add_box({0.0f, 0.0f}, {40.0f, 2.0f}, ZERO, ZERO, ZERO, INFINITY, 0.1f);
        scene.add_box({18.0f, 2.0f}, {22.0f, 3.0f}, ZERO, ZERO, GRAVITY, 1.0f, 0.1f);
        scene.add_box({19.0f, 3.02f}, {21.0f, 4.02f}, ZERO, ZERO, GRAVITY, 1.0f, 0.1f);

        auto& o = scene.get_objects();
        for (int i = 0; i < static_cast<int>(4.0f / DT); i++) scene.step(DT);
        REQUIRE(scene.count_sleeping() == 2);

        const phys::real before = o[2]->position.y;
        scene.remove_object(1);                 // pull the support out

        for (int i = 0; i < static_cast<int>(2.0f / DT); i++) scene.step(DT);

        REQUIRE(scene.get_objects()[1]->position.y < before - 0.5f);
    }
}

TEST_CASE("Oriented boxes")
{
    SECTION("a box dropped flat on a plane rests level and stays level")
    {
        Scene scene{40.0f, 40.0f};
        scene.add_plane({0.0f, 5.0f}, {0.0f, 1.0f}, 0.1f);
        scene.add_box({9.0f, 8.0f}, {11.0f, 9.0f}, ZERO, ZERO, GRAVITY, 1.0f, 0.1f);

        auto* b = scene.get_objects()[1].get();
        for (int i = 0; i < static_cast<int>(6.0f / DT); i++) scene.step(DT);

        REQUIRE(b->position.y == Catch::Approx(5.5f).margin(0.02f));
        REQUIRE(b->orientation == Catch::Approx(0.0f).margin(0.001f));
        REQUIRE(b->angular_velocity == Catch::Approx(0.0f).margin(0.001f));
    }

    SECTION("a tall box stands up on its own")
    {
        Scene scene{40.0f, 40.0f};
        scene.add_box({0.0f, 0.0f}, {40.0f, 2.0f}, ZERO, ZERO, ZERO, INFINITY, 0.1f);
        scene.add_box({19.7f, 2.0f}, {20.3f, 5.0f}, ZERO, ZERO, GRAVITY, 1.0f, 0.1f);

        auto& o = scene.get_objects();
        o[0]->friction = 0.6f;
        o[1]->friction = 0.6f;

        for (int i = 0; i < static_cast<int>(4.0f / DT); i++) scene.step(DT);

        REQUIRE(std::fabs(o[1]->orientation) < 0.01f);
    }

    SECTION("a nudged tall box topples and lies down")
    {
        Scene scene{40.0f, 40.0f};
        scene.add_box({0.0f, 0.0f}, {40.0f, 2.0f}, ZERO, ZERO, ZERO, INFINITY, 0.1f);
        scene.add_box({19.7f, 2.0f}, {20.3f, 5.0f}, ZERO, ZERO, GRAVITY, 1.0f, 0.1f);

        auto& o = scene.get_objects();
        o[0]->friction = 0.6f;
        o[1]->friction = 0.6f;

        for (int i = 0; i < static_cast<int>(1.0f / DT); i++) scene.step(DT);
        o[1]->angular_velocity = -2.0f;

        for (int i = 0; i < static_cast<int>(4.0f / DT); i++) scene.step(DT);

        // A quarter turn, then at rest on its long side.
        const phys::real degrees = std::fabs(o[1]->orientation) * 57.29578f;
        REQUIRE(degrees == Catch::Approx(90.0f).margin(3.0f));
        REQUIRE(o[1]->angular_velocity == Catch::Approx(0.0f).margin(0.01f));
    }

    SECTION("a box lying flat on a slope obeys the friction criterion")
    {
        auto slide = [](phys::real degrees, phys::real mu)
        {
            const phys::real theta = degrees * 3.14159265f / 180.0f;
            const phys::Vec2 normal{-std::sin(theta), std::cos(theta)};
            const phys::Vec2 uphill{ std::cos(theta), std::sin(theta)};
            const phys::real half = 0.5f;

            Scene scene{300.0f, 300.0f};
            scene.add_plane({0.0f, 0.0f}, normal, 0.1f);

            const phys::Vec2 centre = uphill * 150.0f + normal * half;
            scene.add_box(centre - phys::Vec2{half, half}, centre + phys::Vec2{half, half},
                          ZERO, ZERO, GRAVITY, 1.0f, 0.1f);

            auto& o = scene.get_objects();
            o[1]->orientation = theta;
            o[0]->friction = mu;
            o[1]->friction = mu;

            const phys::Vec2 start = o[1]->position;
            for (int i = 0; i < static_cast<int>(12.0f / DT); i++) scene.step(DT);

            return phys::Vec2::dot(o[1]->position - start, uphill);
        };

        REQUIRE(std::fabs(slide(30.0f, 0.8f)) < 0.2f);    // mu > tan(30) = 0.577
        REQUIRE(slide(30.0f, 0.2f) < -20.0f);
        REQUIRE(std::fabs(slide(20.0f, 0.5f)) < 0.2f);    // mu > tan(20) = 0.364
        REQUIRE(slide(50.0f, 0.8f) < -20.0f);             // mu < tan(50) = 1.192
    }

    SECTION("a box rotated 45 degrees rests on its corner, higher than flat")
    {
        // Half diagonal of a unit box is sqrt(2)/2 = 0.7071, against a half
        // extent of 0.5 when flat. A collision test that ignored orientation
        // would settle it at the flat height instead.
        Scene scene{40.0f, 40.0f};
        scene.add_plane({0.0f, 5.0f}, {0.0f, 1.0f}, 0.1f);
        scene.add_box({19.5f, 10.0f}, {20.5f, 11.0f}, ZERO, ZERO, GRAVITY, 1.0f, 0.0f);

        auto& o = scene.get_objects();
        o[0]->friction = 0.9f;
        o[1]->friction = 0.9f;
        o[1]->orientation = 3.14159265f / 4.0f;
        o[1]->angular_velocity = 0.0f;

        for (int i = 0; i < static_cast<int>(4.0f / DT); i++) scene.step(DT);

        // It topples onto a face from the corner balance, so it ends lower than
        // the corner height but the orientation must have moved off 45 degrees.
        REQUIRE(o[1]->position.y > 5.0f);
        REQUIRE(o[1]->position.y < 5.0f + 0.7072f);
    }

    SECTION("the bounding box grows when a box rotates")
    {
        phys::Box box{{0.0f, 0.0f}, {2.0f, 1.0f}, ZERO, ZERO, ZERO, 1.0f, 0.5f};

        REQUIRE(box.get_half_body().x == Catch::Approx(1.0f));
        REQUIRE(box.bounds_half().x == Catch::Approx(1.0f));

        box.orientation = 3.14159265f / 2.0f;   // a quarter turn swaps the extents
        REQUIRE(box.bounds_half().x == Catch::Approx(0.5f).margin(1e-4f));
        REQUIRE(box.bounds_half().y == Catch::Approx(1.0f).margin(1e-4f));
    }
}

TEST_CASE("Angular dynamics basics")
{
    SECTION("a free body keeps its spin")
    {
        Scene scene{100.0f, 100.0f};
        scene.add_circle({50.0f, 50.0f}, ZERO, ZERO, 0.5f, ZERO, 1.0f, 0.5f);

        auto* c = scene.get_objects()[0].get();
        c->angular_velocity = 3.0f;

        const int steps = static_cast<int>(1.0f / DT);
        for (int i = 0; i < steps; i++) scene.step(DT);

        REQUIRE(c->angular_velocity == Catch::Approx(3.0f));
        REQUIRE(c->orientation == Catch::Approx(3.0f * steps * DT).margin(0.001f));
    }

    SECTION("torque produces angular acceleration of torque over inertia")
    {
        // Solid disc, m = 2, r = 0.5 -> I = m r^2 / 2 = 0.25.
        Scene scene{100.0f, 100.0f};
        scene.add_circle({50.0f, 50.0f}, ZERO, ZERO, 0.5f, ZERO, 2.0f, 0.5f);

        auto* c = scene.get_objects()[0].get();
        c->torque = 0.5f;   // alpha = 0.5 / 0.25 = 2 rad/s^2

        // Against simulated time, not nominal: 1.0f/DT truncates to 119 steps,
        // not 120, so a nominal one-second loop is a step short.
        const int steps = static_cast<int>(1.0f / DT);
        for (int i = 0; i < steps; i++) scene.step(DT);

        REQUIRE(c->angular_velocity == Catch::Approx(2.0f * steps * DT).margin(0.001f));
    }

    SECTION("a box spins under torque, with rectangle inertia")
    {
        // Rectangle about its centre: I = m (w^2 + h^2) / 12. For m = 1,
        // w = 2, h = 1 that is 5/12, so alpha = torque / I.
        Scene scene{100.0f, 100.0f};
        scene.add_box({10.0f, 10.0f}, {12.0f, 11.0f}, ZERO, ZERO, ZERO, 1.0f, 0.5f);

        auto* b = scene.get_objects()[0].get();
        REQUIRE(b->inv_inertia == Catch::Approx(12.0f / 5.0f));

        b->torque = 1.0f;
        const int steps = static_cast<int>(1.0f / DT);
        for (int i = 0; i < steps; i++) scene.step(DT);

        REQUIRE(b->angular_velocity == Catch::Approx(2.4f * steps * DT).margin(0.001f));
    }

    SECTION("a static disc has no inverse inertia")
    {
        Scene scene{100.0f, 100.0f};
        scene.add_circle({50.0f, 50.0f}, ZERO, ZERO, 0.5f, ZERO, INFINITY, 0.5f);

        REQUIRE(scene.get_objects()[0]->inv_inertia == 0.0f);
    }

    SECTION("an off-centre impact starts a disc spinning")
    {
        // Dropped so it strikes the corner of a box rather than its face.
        Scene scene{40.0f, 40.0f};
        scene.add_box({10.0f, 0.0f}, {20.0f, 5.0f}, ZERO, ZERO, ZERO, INFINITY, 0.3f);
        scene.add_circle({20.3f, 10.0f}, ZERO, ZERO, 0.5f, GRAVITY, 1.0f, 0.3f);

        auto* c = scene.get_objects()[1].get();
        for (int i = 0; i < static_cast<int>(3.0f / DT); i++) scene.step(DT);

        REQUIRE(std::fabs(c->angular_velocity) > 0.01f);
    }

    SECTION("a spinning body does not fall asleep")
    {
        Scene scene{40.0f, 40.0f};
        scene.add_box({0.0f, 0.0f}, {40.0f, 2.0f}, ZERO, ZERO, ZERO, INFINITY, 0.2f);
        scene.add_circle({20.0f, 2.5f}, ZERO, ZERO, 0.5f, GRAVITY, 1.0f, 0.2f);

        auto* c = scene.get_objects()[1].get();
        for (int i = 0; i < static_cast<int>(3.0f / DT); i++)
        {
            scene.step(DT);
            c->angular_velocity = 5.0f;   // held spinning
        }

        REQUIRE(scene.count_sleeping() == 0);
    }
}

TEST_CASE("A disc rolls down a ramp")
{
    // These cases used to assert that friction could hold a circle on a ramp.
    // That was never true physics and only passed because circles could not
    // rotate: static friction prevents slipping, not rolling, so a disc rolls
    // down any incline whatever the friction. Now that rotation exists, the
    // right assertion is the rolling acceleration.
    auto roll = [](phys::real degrees, phys::real mu, phys::real seconds)
    {
        const phys::real theta = degrees * 3.14159265f / 180.0f;
        const phys::Vec2 normal{-std::sin(theta), std::cos(theta)};
        const phys::Vec2 uphill{ std::cos(theta), std::sin(theta)};
        const phys::real radius = 0.5f;

        Scene scene{200.0f, 200.0f};
        scene.add_plane({0.0f, 0.0f}, normal, 0.0f);
        scene.add_circle(uphill * 100.0f + normal * radius, ZERO, ZERO, radius,
                         GRAVITY, 1.0f, 0.0f);

        auto& o = scene.get_objects();
        o[0]->friction = mu;
        o[1]->friction = mu;

        // A finer step than the usual 1/120: the analytic comparison is to a
        // continuous solution, so integration error shows up directly.
        const phys::real h = 1.0f / 480.0f;
        for (int i = 0; i < static_cast<int>(seconds / h); i++) scene.step(h);

        const phys::real along = phys::Vec2::dot(o[1]->velocity, uphill);
        struct R { phys::real accel; phys::real omega; phys::real slip; };
        return R{-along / seconds, o[1]->angular_velocity,
                 std::fabs(along) - std::fabs(o[1]->angular_velocity * radius)};
    };

    const phys::real g_sin = 9.8f * std::sin(30.0f * 3.14159265f / 180.0f);

    SECTION("with no friction it slides without spinning")
    {
        const auto r = roll(30.0f, 0.0f, 2.0f);

        REQUIRE(r.accel == Catch::Approx(g_sin).margin(0.05f));
        REQUIRE(r.omega == Catch::Approx(0.0f).margin(1e-4f));
    }

    SECTION("with enough friction it rolls without slipping")
    {
        // A solid disc has I = m r^2 / 2, so a third of the work goes into
        // spin and the acceleration is g sin(theta) / 1.5.
        const auto r = roll(30.0f, 0.5f, 2.0f);

        REQUIRE(r.accel == Catch::Approx(g_sin / 1.5f).margin(0.05f));
        REQUIRE(r.slip == Catch::Approx(0.0f).margin(0.01f));   // contact is stationary
        REQUIRE(r.omega > 0.0f);
    }

    SECTION("rolling acceleration does not depend on friction above the threshold")
    {
        // Once rolling, static friction supplies whatever torque is needed, so
        // mu drops out of the answer entirely.
        const auto low  = roll(30.0f, 0.5f, 2.0f);
        const auto high = roll(30.0f, 0.9f, 2.0f);

        REQUIRE(low.accel == Catch::Approx(high.accel).margin(0.001f));
        REQUIRE(low.omega == Catch::Approx(high.omega).margin(0.001f));
    }

    SECTION("too little friction leaves it slipping as it spins up")
    {
        const auto r = roll(30.0f, 0.1f, 2.0f);

        REQUIRE(r.omega > 0.0f);                 // it does spin
        REQUIRE(r.slip > 0.1f);                  // but the contact still slides
        REQUIRE(r.accel > g_sin / 1.5f);         // so it outruns pure rolling
        REQUIRE(r.accel < g_sin);
    }
}

TEST_CASE("Friction brings a sliding box to rest")
{
    // Boxes only: a Box cannot rotate, so sliding friction is the whole
    // story for one. The disc cases moved to the rolling test above.
    auto slide_distance = [](phys::real mu, bool use_circle)
    {
        Scene scene{200.0f, 40.0f};
        add_floor(scene, 0.1f);

        if (use_circle) scene.add_circle({5.0f, 2.5f}, {12.0f, 0.0f}, ZERO, 0.5f, GRAVITY, 1.0f, 0.1f);
        else            scene.add_box({5.0f, 2.0f}, {6.0f, 3.0f}, {12.0f, 0.0f}, ZERO, GRAVITY, 1.0f, 0.1f);

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

    SECTION("stopping distance matches v^2 / (2 mu g), for a body that cannot spin")
    {
        const auto [measured, v] = slide_distance(0.3f, false);
        const phys::real predicted = (12.0f * 12.0f) / (2.0f * 0.3f * 9.8f);

        REQUIRE(measured == Catch::Approx(predicted).margin(0.5f));
    }

    SECTION("a disc does not stop like a box: it spins up and keeps rolling")
    {
        // This used to assert circle and box behaved identically, which only
        // held while circles could not rotate. A disc converts sliding into
        // spin and then rolls, and no rolling resistance is modelled, so it
        // keeps going where a box stops.
        const auto [box, v_box]       = slide_distance(0.3f, false);
        const auto [circle, v_circle] = slide_distance(0.3f, true);

        REQUIRE(v_box == Catch::Approx(0.0f).margin(0.01f));
        REQUIRE(v_circle > 1.0f);
        REQUIRE(circle > box * 2.0f);
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
    scene.add_box({9.0f, 6.0f}, {11.0f, 7.0f}, ZERO, ZERO, GRAVITY, 1.0f, 0.9f);

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
        scene.add_box({9.0f, 2.02f + 1.0f * i}, {11.0f, 3.02f + 1.0f * i},
                       ZERO, ZERO, GRAVITY, 1.0f, 0.1f);

    SECTION("a stack sleeps, and every one of its contacts is skipped")
    {
        for (int i = 0; i < static_cast<int>(3.0f / DT); i++) scene.step(DT);

        REQUIRE(scene.count_sleeping() == 5);

        // One more step, now that everything is known to be asleep. The counter
        // only covers the most recent step, and update_sleep runs at the end of
        // one, so the step where bodies first fall asleep processed its contacts
        // while they were still awake. Asserting straight after the settle loop
        // passed on Windows and Linux and failed on macOS purely on timing.
        scene.step(DT);

        REQUIRE(scene.get_contacts_skipped_asleep() == scene.get_contacts().size());
        REQUIRE(scene.get_contacts_skipped_asleep() > 0);
    }

    SECTION("an impact wakes the island, which then re-sleeps")
    {
        for (int i = 0; i < static_cast<int>(3.0f / DT); i++) scene.step(DT);
        REQUIRE(scene.count_sleeping() == 5);

        scene.add_box({9.0f, 12.0f}, {11.0f, 13.0f}, {0.0f, -8.0f}, ZERO, GRAVITY, 1.0f, 0.1f);

        bool woke = false;
        for (int i = 0; i < static_cast<int>(1.0f / DT); i++)
        {
            scene.step(DT);
            if (scene.count_sleeping() == 0) woke = true;
        }
        REQUIRE(woke);

        for (int i = 0; i < static_cast<int>(4.0f / DT); i++) scene.step(DT);

        REQUIRE(scene.count_sleeping() == 6);

        scene.step(DT);   // see the note above: the counter covers one step

        REQUIRE(scene.get_contacts_skipped_asleep() == scene.get_contacts().size());
        REQUIRE(scene.get_contacts_skipped_asleep() > 0);
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
