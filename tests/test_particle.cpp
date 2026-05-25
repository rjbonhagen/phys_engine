#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "phys/Particle.hpp"

TEST_CASE("Particle")
{
    SECTION("default constructor sets mass")
    {
        phys::Particle p{};
        REQUIRE(p.mass == Catch::Approx(0.001f));
    }

    SECTION("default constructor sets radius")
    {
        phys::Particle p{};
        REQUIRE(p.radius == Catch::Approx(0.01f));
    }

    SECTION("default constructor zeroes kinematic fields")
    {
        phys::Particle p{};
        REQUIRE(p.position == phys::Vec2{});
        REQUIRE(p.velocity == phys::Vec2{});
        REQUIRE(p.acceleration == phys::Vec2{});
        REQUIRE(p.forces == phys::Vec2{});
    }
}
