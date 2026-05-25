#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "phys/Circle.hpp"

TEST_CASE("Circle")
{
    SECTION("default constructor zeroes all fields including radius")
    {
        phys::Circle c{};
        REQUIRE(c.position == phys::Vec2{});
        REQUIRE(c.velocity == phys::Vec2{});
        REQUIRE(c.acceleration == phys::Vec2{});
        REQUIRE(c.forces == phys::Vec2{});
        REQUIRE(c.mass == 0.0f);
        REQUIRE(c.radius == 0.0f);
    }

    SECTION("constructor with parameters")
    {
        phys::Circle c{{1.0f, 2.0f}, {3.0f, 4.0f}, {5.0f, 6.0f}, {7.0f, 8.0f}, 9.0f, 0.5f};
        REQUIRE(c.position == phys::Vec2{1.0f, 2.0f});
        REQUIRE(c.mass == 9.0f);
        REQUIRE(c.radius == Catch::Approx(0.5f));
    }

    SECTION("radius is independent of position")
    {
        phys::Circle a{{0.0f, 0.0f}, {}, {}, {}, 1.0f, 2.0f};
        phys::Circle b{{5.0f, 5.0f}, {}, {}, {}, 1.0f, 2.0f};
        REQUIRE(a.radius == Catch::Approx(b.radius));
    }
}
