#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "Scene.hpp"
#include "phys/Circle.hpp"

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
