#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "Scene.hpp"
#include "phys/Circle.hpp"
#include <algorithm>
#include <cmath>

// helpers
static phys::Circle& as_circle(phys::Object& o) { return static_cast<phys::Circle&>(o); }

TEST_CASE("Scene integration")
{
    SECTION("constant velocity moves position by v*dt each step")
    {
        // forces = 0 so acceleration = 0, velocity stays constant
        Scene scene{1000, 1000};
        scene.add_circle({100.0f, 100.0f}, {20.0f, 0.0f}, {}, 5.0f, {}, 1.0f, 0.5f);
        scene.step(0.1f);
        auto& c = as_circle(*scene.get_objects()[0]);
        REQUIRE(c.position.x == Catch::Approx(102.0f));
        REQUIRE(c.position.y == Catch::Approx(100.0f));
    }

    SECTION("force generates acceleration which updates velocity and position")
    {
        // F=10, m=1 → a=10; after dt=0.1: v=1, x+=0.1
        Scene scene{1000, 1000};
        scene.add_circle({100.0f, 100.0f}, {}, {}, 5.0f, {10.0f, 0.0f}, 1.0f, 0.5f);
        scene.step(0.1f);
        auto& c = as_circle(*scene.get_objects()[0]);
        REQUIRE(c.velocity.x == Catch::Approx(1.0f));
        REQUIRE(c.position.x == Catch::Approx(100.1f));
    }

    SECTION("multiple steps accumulate correctly")
    {
        Scene scene{1000, 1000};
        scene.add_circle({100.0f, 100.0f}, {10.0f, 0.0f}, {}, 5.0f, {}, 1.0f, 0.5f);
        scene.step(0.1f);
        scene.step(0.1f);
        auto& c = as_circle(*scene.get_objects()[0]);
        REQUIRE(c.position.x == Catch::Approx(102.0f));
    }
}

TEST_CASE("Scene border collision")
{
    SECTION("circle clamped to left wall and x-velocity flipped")
    {
        // After integrate: x = 4 - 5 = -1 (past left wall at x=0, radius=5)
        Scene scene{400, 400};
        scene.add_circle({4.0f, 200.0f}, {-100.0f, 0.0f}, {}, 5.0f, {}, 1.0f, 1.0f);
        scene.step(0.1f);
        auto& c = as_circle(*scene.get_objects()[0]);
        REQUIRE(c.position.x == Catch::Approx(5.0f));
        REQUIRE(c.velocity.x > 0.0f);
    }

    SECTION("circle clamped to right wall and x-velocity flipped")
    {
        // After integrate: x = 396 + 100*0.1 = 406, past right wall at 400, radius=5
        Scene scene{400, 400};
        scene.add_circle({396.0f, 200.0f}, {100.0f, 0.0f}, {}, 5.0f, {}, 1.0f, 1.0f);
        scene.step(0.1f);
        auto& c = as_circle(*scene.get_objects()[0]);
        REQUIRE(c.position.x == Catch::Approx(395.0f));
        REQUIRE(c.velocity.x < 0.0f);
    }

    SECTION("circle clamped to top wall and y-velocity flipped")
    {
        Scene scene{400, 400};
        scene.add_circle({200.0f, 4.0f}, {0.0f, -100.0f}, {}, 5.0f, {}, 1.0f, 1.0f);
        scene.step(0.1f);
        auto& c = as_circle(*scene.get_objects()[0]);
        REQUIRE(c.position.y == Catch::Approx(5.0f));
        REQUIRE(c.velocity.y > 0.0f);
    }

    SECTION("circle clamped to bottom wall and y-velocity flipped")
    {
        Scene scene{400, 400};
        scene.add_circle({200.0f, 396.0f}, {0.0f, 100.0f}, {}, 5.0f, {}, 1.0f, 1.0f);
        scene.step(0.1f);
        auto& c = as_circle(*scene.get_objects()[0]);
        REQUIRE(c.position.y == Catch::Approx(395.0f));
        REQUIRE(c.velocity.y < 0.0f);
    }

    SECTION("circle moving away from wall is not reflected")
    {
        Scene scene{400, 400};
        scene.add_circle({200.0f, 200.0f}, {10.0f, 0.0f}, {}, 5.0f, {}, 1.0f, 1.0f);
        scene.step(0.1f);
        auto& c = as_circle(*scene.get_objects()[0]);
        REQUIRE(c.velocity.x > 0.0f);
    }
}

TEST_CASE("Scene circle-circle collision")
{
    SECTION("two circles heading toward each other separate after step")
    {
        // A at x=45 moving right +50, B at x=55 moving left -50; radii=5 → touching at x=50
        // After integrate they overlap; collision response reverses the x components
        Scene scene{400, 400};
        scene.add_circle({45.0f, 200.0f}, {50.0f, 0.0f}, {}, 5.0f, {}, 1.0f, 1.0f);
        scene.add_circle({55.0f, 200.0f}, {-50.0f, 0.0f}, {}, 5.0f, {}, 1.0f, 1.0f);
        scene.step(0.01f);
        auto& a = as_circle(*scene.get_objects()[0]);
        auto& b = as_circle(*scene.get_objects()[1]);
        // after impulse, A should move left (or slower right) and B right (or slower left)
        REQUIRE(a.velocity.x < 50.0f);
        REQUIRE(b.velocity.x > -50.0f);
    }

    SECTION("two circles moving apart do not affect each other's velocity")
    {
        // A at x=200 moving left, B at x=300 moving right — they diverge, no collision
        Scene scene{800, 800};
        scene.add_circle({200.0f, 400.0f}, {-10.0f, 0.0f}, {}, 5.0f, {}, 1.0f, 0.5f);
        scene.add_circle({300.0f, 400.0f}, {10.0f, 0.0f}, {}, 5.0f, {}, 1.0f, 0.5f);
        scene.step(0.1f);
        auto& a = as_circle(*scene.get_objects()[0]);
        auto& b = as_circle(*scene.get_objects()[1]);
        REQUIRE(a.velocity.x == Catch::Approx(-10.0f));
        REQUIRE(b.velocity.x == Catch::Approx(10.0f));
    }

    SECTION("elastic collision with equal masses exchanges velocities")
    {
        // circles start at distance=10 (touching); after integrate they overlap → collision fires
        // restitution product = 1.0 → elastic; equal mass head-on → velocities swap
        Scene scene{400, 400};
        scene.add_circle({195.0f, 200.0f}, {10.0f, 0.0f}, {}, 5.0f, {}, 1.0f, 1.0f);
        scene.add_circle({205.0f, 200.0f}, {-10.0f, 0.0f}, {}, 5.0f, {}, 1.0f, 1.0f);
        scene.step(0.01f);
        auto& a = as_circle(*scene.get_objects()[0]);
        auto& b = as_circle(*scene.get_objects()[1]);
        REQUIRE(a.velocity.x == Catch::Approx(-10.0f).margin(0.5f));
        REQUIRE(b.velocity.x == Catch::Approx(10.0f).margin(0.5f));
    }
}

TEST_CASE("Scene circle-Box collision")
{
    const phys::Vec2 ZERO{0.0f, 0.0f};
    const phys::Vec2 GRAVITY{0.0f, -9.8f};

    SECTION("circle comes to rest on top of a static box")
    {
        // Box top face at y = 2, circle radius 0.5 -> resting centre at y = 2.5,
        // less the 0.01 penetration slop the solver leaves in place.
        Scene scene{20.0f, 20.0f};
        scene.add_box({0.0f, 0.0f}, {20.0f, 2.0f}, ZERO, ZERO, ZERO, INFINITY, 1.0f);
        scene.add_circle({10.0f, 8.0f}, ZERO, ZERO, 0.5f, GRAVITY, 1.0f, 0.5f);

        for (int i = 0; i < 300; i++) scene.step(1.0f / 60.0f);

        auto& c = as_circle(*scene.get_objects()[1]);
        REQUIRE(c.position.y == Catch::Approx(2.5f).margin(0.05f));
    }

    SECTION("a static box is not moved by a circle landing on it")
    {
        Scene scene{20.0f, 20.0f};
        scene.add_box({0.0f, 0.0f}, {20.0f, 2.0f}, ZERO, ZERO, ZERO, INFINITY, 1.0f);
        scene.add_circle({10.0f, 8.0f}, ZERO, ZERO, 0.5f, GRAVITY, 1.0f, 0.5f);

        const phys::Vec2 before = scene.get_objects()[0]->position;
        for (int i = 0; i < 300; i++) scene.step(1.0f / 60.0f);

        REQUIRE(scene.get_objects()[0]->position == before);
        REQUIRE(scene.get_objects()[0]->velocity == ZERO);
    }

    SECTION("a fast circle does not pass through a box")
    {
        // Without the speed clamp a circle this fast clears the whole box in one
        // step and is never detected.
        Scene scene{20.0f, 40.0f};
        scene.add_box({0.0f, 0.0f}, {20.0f, 2.0f}, ZERO, ZERO, ZERO, INFINITY, 1.0f);
        scene.add_circle({10.0f, 10.0f}, {0.0f, -5000.0f}, ZERO, 0.5f, ZERO, 1.0f, 0.5f);

        for (int i = 0; i < 180; i++) scene.step(1.0f / 60.0f);

        auto& c = as_circle(*scene.get_objects()[1]);
        REQUIRE(c.position.y > 2.0f);
    }

    SECTION("a circle that penetrates past the box midline exits the way it came")
    {
        // A thin wall (1 unit thick, midline y = 0.5) and a small, fast circle:
        // one step carries its centre past the midline, so the shortest way out
        // is through the bottom. It must still be pushed back up the way it
        // entered, not ejected through the wall.
        Scene scene{20.0f, 40.0f};
        scene.add_box({0.0f, 0.0f}, {20.0f, 1.0f}, ZERO, ZERO, ZERO, INFINITY, 1.0f);
        scene.add_circle({10.0f, 1.25f}, {0.0f, -50.0f}, ZERO, 0.2f, ZERO, 1.0f, 0.5f);

        scene.step(1.0f / 60.0f);

        auto& c = as_circle(*scene.get_objects()[1]);
        REQUIRE(c.velocity.y > 0.0f);   // reflected upward, not driven through

        for (int i = 0; i < 60; i++) scene.step(1.0f / 60.0f);
        REQUIRE(c.position.y > 1.0f);   // ended up above the wall
    }

    SECTION("circle hitting a box side is reflected horizontally")
    {
        Scene scene{40.0f, 20.0f};
        scene.add_box({10.0f, 0.0f}, {12.0f, 20.0f}, ZERO, ZERO, ZERO, INFINITY, 1.0f);
        scene.add_circle({8.0f, 10.0f}, {20.0f, 0.0f}, ZERO, 0.5f, ZERO, 1.0f, 0.5f);

        for (int i = 0; i < 30; i++) scene.step(1.0f / 60.0f);

        auto& c = as_circle(*scene.get_objects()[1]);
        REQUIRE(c.velocity.x < 0.0f);
        REQUIRE(c.position.x < 10.0f);
    }

    SECTION("a circle resting on a dynamic box does not gain energy")
    {
        // The runaway case: a box bouncing on an elastic floor repeatedly
        // slingshots the circle above it. With restitution below 1 both settle.
        Scene scene{16.0f, 9.6f};
        scene.add_box({-1.0f, -1.0f}, {17.0f, 0.0f}, ZERO, ZERO, ZERO, INFINITY, 1.0f);
        scene.add_box({7.0f, 3.0f}, {9.0f, 4.0f}, ZERO, ZERO, GRAVITY, 1.0f, 0.6f);
        scene.add_circle({8.0f, 7.0f}, ZERO, ZERO, 0.2f, GRAVITY, 1.0f, 0.7f);

        phys::real peak = 0.0f;
        for (int i = 0; i < 60 * 60; i++)
        {
            scene.step(1.0f / 60.0f);
            for (size_t k = 1; k < scene.get_objects().size(); k++)
                peak = std::max(peak, scene.get_objects()[k]->velocity.length());
        }

        // Free fall from y = 7 onto the floor caps out around 11 units/s.
        REQUIRE(peak < 15.0f);
    }
}
