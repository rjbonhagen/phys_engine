#include <catch2/catch_test_macros.hpp>
#include "phys/math/Vec2.hpp"

TEST_CASE("Vec2 operations")
{
    phys::Vec2 a(1.0f, 2.0f);

    SECTION("x is correct") { REQUIRE(a.x == 1.0f); }
    SECTION("y is correct") { REQUIRE(a.y == 2.0f); }
}
