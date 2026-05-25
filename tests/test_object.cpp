#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "phys/Object.hpp"

TEST_CASE("Object")
{
    SECTION("default constructor zeroes all fields")
    {
        phys::Object o{};
        REQUIRE(o.position == phys::Vec2{});
        REQUIRE(o.velocity == phys::Vec2{});
        REQUIRE(o.acceleration == phys::Vec2{});
        REQUIRE(o.forces == phys::Vec2{});
        REQUIRE(o.mass == 0.0f);
    }

    SECTION("constructor with parameters")
    {
        phys::Object o{{1.0f, 2.0f}, {3.0f, 4.0f}, {5.0f, 6.0f}, {7.0f, 8.0f}, 9.0f};
        REQUIRE(o.position == phys::Vec2{1.0f, 2.0f});
        REQUIRE(o.velocity == phys::Vec2{3.0f, 4.0f});
        REQUIRE(o.acceleration == phys::Vec2{5.0f, 6.0f});
        REQUIRE(o.forces == phys::Vec2{7.0f, 8.0f});
        REQUIRE(o.mass == 9.0f);
    }

    SECTION("operator== equal objects")
    {
        phys::Object a{{1.0f, 2.0f}, {3.0f, 4.0f}, {5.0f, 6.0f}, {7.0f, 8.0f}, 9.0f};
        phys::Object b{{1.0f, 2.0f}, {3.0f, 4.0f}, {5.0f, 6.0f}, {7.0f, 8.0f}, 9.0f};
        REQUIRE(a == b);
    }

    SECTION("operator== different position")
    {
        phys::Object a{{1.0f, 2.0f}, {}, {}, {}, 1.0f};
        phys::Object b{{9.0f, 9.0f}, {}, {}, {}, 1.0f};
        REQUIRE_FALSE(a == b);
    }

    SECTION("operator== different mass")
    {
        phys::Object a{{}, {}, {}, {}, 1.0f};
        phys::Object b{{}, {}, {}, {}, 2.0f};
        REQUIRE_FALSE(a == b);
    }
}
