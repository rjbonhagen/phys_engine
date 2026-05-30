#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "phys/Circle.hpp"

TEST_CASE("Circle")
{
    SECTION("constructor stores all fields")
    {
        phys::Circle c{{1.0f, 2.0f}, {3.0f, 4.0f}, {5.0f, 6.0f}, {7.0f, 8.0f}, 9.0f, 0.5f, 10.0f};
        REQUIRE(c.position == phys::Vec2{1.0f, 2.0f});
        REQUIRE(c.velocity == phys::Vec2{3.0f, 4.0f});
        REQUIRE(c.acceleration == phys::Vec2{5.0f, 6.0f});
        REQUIRE(c.forces == phys::Vec2{7.0f, 8.0f});
        REQUIRE(c.mass == 9.0f);
        REQUIRE(c.restitution == Catch::Approx(0.5f));
        REQUIRE(c.radius == Catch::Approx(10.0f));
    }

    SECTION("radius is independent of position")
    {
        phys::Circle a{{0.0f, 0.0f}, {}, {}, {}, 1.0f, 0.5f, 5.0f};
        phys::Circle b{{100.0f, 100.0f}, {}, {}, {}, 1.0f, 0.5f, 5.0f};
        REQUIRE(a.radius == Catch::Approx(b.radius));
    }

    SECTION("is-a Object — operator== ignores radius")
    {
        phys::Circle a{{1.0f, 0.0f}, {}, {}, {}, 1.0f, 0.5f, 5.0f};
        phys::Circle b{{1.0f, 0.0f}, {}, {}, {}, 1.0f, 0.5f, 9.0f};
        REQUIRE(static_cast<phys::Object&>(a) == static_cast<phys::Object&>(b));
    }
}
