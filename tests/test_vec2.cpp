#include <catch2/catch_test_macros.hpp>
#include "phys/math/Vec2.hpp"

TEST_CASE("Vec2")
{

    SECTION("Empty Constructor") 
    { 
        phys::Vec2 a{};
        REQUIRE(a.x == 0.0f); 
        REQUIRE(a.y == 0.0f);
    }
    SECTION("Constructor w/ parameters ")
    {
        phys::Vec2 a(6.7f, 3.2f);
        REQUIRE(a.x == 6.7f);
        REQUIRE(a.y == 3.2f);
    }

    SECTION("operator+ basic")
    {
        phys::Vec2 a(1.0f, 2.0f);
        phys::Vec2 b(3.0f, 4.0f);
        phys::Vec2 c = a + b;
        REQUIRE(c.x == 4.0f);
        REQUIRE(c.y == 6.0f);
    }

    SECTION("operator+ with zero vector")
    {
        phys::Vec2 a(5.0f, -3.0f);
        phys::Vec2 zero{};
        phys::Vec2 c = a + zero;
        REQUIRE(c.x == 5.0f);
        REQUIRE(c.y == -3.0f);
    }

    SECTION("operator+ with negative components")
    {
        phys::Vec2 a(-1.0f, -2.0f);
        phys::Vec2 b(-3.0f, -4.0f);
        phys::Vec2 c = a + b;
        REQUIRE(c.x == -4.0f);
        REQUIRE(c.y == -6.0f);
    }

    SECTION("operator+ does not modify operands")
    {
        phys::Vec2 a(1.0f, 2.0f);
        phys::Vec2 b(3.0f, 4.0f);
        a + b;
        REQUIRE(a.x == 1.0f);
        REQUIRE(a.y == 2.0f);
        REQUIRE(b.x == 3.0f);
        REQUIRE(b.y == 4.0f);
    }

}
