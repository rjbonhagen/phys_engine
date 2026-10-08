#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <cmath>

#include "Viewport.hpp"

TEST_CASE("Viewport maps world to screen")
{
    SECTION("an exactly fitting window puts the world corners at the window corners")
    {
        // 16 x 9.6 metres at 50 px/m is exactly 800 x 480.
        Viewport v{800, 480, 16.0f, 9.6f};

        REQUIRE(v.pixels_per_metre() == Catch::Approx(50.0f));

        // World y is up, screen y is down, so world (0,0) is bottom-left.
        const phys::Vec2 bottom_left = v.to_screen({0.0f, 0.0f});
        // Approx(0) is a relative test, useless at zero: needs a margin.
        REQUIRE(bottom_left.x == Catch::Approx(0.0f).margin(1e-3f));
        REQUIRE(bottom_left.y == Catch::Approx(480.0f).margin(1e-3f));

        const phys::Vec2 top_right = v.to_screen({16.0f, 9.6f});
        REQUIRE(top_right.x == Catch::Approx(800.0f).margin(1e-3f));
        REQUIRE(top_right.y == Catch::Approx(0.0f).margin(1e-3f));
    }

    SECTION("screen and world round-trip")
    {
        Viewport v{1024, 600, 20.0f, 12.0f};

        for (phys::Vec2 p : {phys::Vec2{0.0f, 0.0f}, phys::Vec2{7.5f, 3.25f},
                             phys::Vec2{20.0f, 12.0f}, phys::Vec2{-2.0f, 15.0f}})
        {
            const phys::Vec2 back = v.to_world(v.to_screen(p));
            REQUIRE(back.x == Catch::Approx(p.x).margin(1e-3f));
            REQUIRE(back.y == Catch::Approx(p.y).margin(1e-3f));
        }
    }
}

TEST_CASE("Viewport preserves the world under resize")
{
    SECTION("a world point keeps its world coordinates when the window changes")
    {
        Viewport v{800, 480, 16.0f, 9.6f};
        const phys::Vec2 point{4.0f, 2.0f};

        const phys::Vec2 before = v.to_world(v.to_screen(point));
        v.set_window(1600, 900);
        const phys::Vec2 after = v.to_world(v.to_screen(point));

        REQUIRE(after.x == Catch::Approx(before.x).margin(1e-3f));
        REQUIRE(after.y == Catch::Approx(before.y).margin(1e-3f));
    }

    SECTION("scale follows the tighter axis, so the world always fits")
    {
        // A wide window against a square world: height is the constraint.
        Viewport wide{2000, 500, 10.0f, 10.0f};
        REQUIRE(wide.pixels_per_metre() == Catch::Approx(50.0f));

        // And the other way round.
        Viewport tall{500, 2000, 10.0f, 10.0f};
        REQUIRE(tall.pixels_per_metre() == Catch::Approx(50.0f));
    }

    SECTION("the world is centred, with equal letterbox margins")
    {
        // Square world in a wide window: 10 m at 50 px/m is 500 px, leaving
        // 750 px of margin each side of a 2000 px window.
        Viewport v{2000, 500, 10.0f, 10.0f};

        const phys::Vec2 left  = v.to_screen({0.0f, 0.0f});
        const phys::Vec2 right = v.to_screen({10.0f, 0.0f});

        REQUIRE(left.x == Catch::Approx(750.0f));
        REQUIRE(2000.0f - right.x == Catch::Approx(750.0f));
    }

    SECTION("one scale for both axes, so a circle stays round")
    {
        Viewport v{1300, 700, 16.0f, 9.6f};

        // Equal world distances must map to equal pixel distances either way.
        const phys::real dx = v.to_screen({1.0f, 0.0f}).x - v.to_screen({0.0f, 0.0f}).x;
        const phys::real dy = v.to_screen({0.0f, 0.0f}).y - v.to_screen({0.0f, 1.0f}).y;

        REQUIRE(dx == Catch::Approx(dy));
    }

    SECTION("changing the world size rescales rather than shifting the view")
    {
        Viewport v{800, 480, 16.0f, 9.6f};
        REQUIRE(v.pixels_per_metre() == Catch::Approx(50.0f));

        v.set_world(32.0f, 19.2f);   // twice as much world in the same window
        REQUIRE(v.pixels_per_metre() == Catch::Approx(25.0f));
        REQUIRE(v.world_size().x == Catch::Approx(32.0f));
    }
}

TEST_CASE("Viewport handles degenerate sizes")
{
    SECTION("a collapsed window does not divide by zero")
    {
        Viewport v{800, 480, 16.0f, 9.6f};

        v.set_window(0, 0);
        const phys::Vec2 p = v.to_screen({1.0f, 1.0f});

        REQUIRE(std::isfinite(p.x));
        REQUIRE(std::isfinite(p.y));
        REQUIRE(v.pixels_per_metre() > 0.0f);
    }

    SECTION("a zero world size falls back rather than producing infinities")
    {
        Viewport v{800, 480, 0.0f, 0.0f};

        REQUIRE(v.pixels_per_metre() == Catch::Approx(1.0f));

        const phys::Vec2 p = v.to_world({400.0f, 240.0f});
        REQUIRE(std::isfinite(p.x));
        REQUIRE(std::isfinite(p.y));
    }
}
