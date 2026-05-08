#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
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

    SECTION("operator- basic")
    {
        phys::Vec2 a(5.0f, 6.0f);
        phys::Vec2 b(3.0f, 2.0f);
        phys::Vec2 c = a - b;
        REQUIRE(c.x == 2.0f);
        REQUIRE(c.y == 4.0f);
    }

    SECTION("operator- with zero vector")
    {
        phys::Vec2 a(5.0f, -3.0f);
        phys::Vec2 zero{};
        phys::Vec2 c = a - zero;
        REQUIRE(c.x == 5.0f);
        REQUIRE(c.y == -3.0f);
    }

    SECTION("operator- self")
    {
        phys::Vec2 a(4.0f, 7.0f);
        phys::Vec2 c = a - a;
        REQUIRE(c.x == 0.0f);
        REQUIRE(c.y == 0.0f);
    }

    SECTION("operator- negative result")
    {
        phys::Vec2 a(1.0f, 2.0f);
        phys::Vec2 b(4.0f, 6.0f);
        phys::Vec2 c = a - b;
        REQUIRE(c.x == -3.0f);
        REQUIRE(c.y == -4.0f);
    }

    SECTION("operator- does not modify operands")
    {
        phys::Vec2 a(5.0f, 6.0f);
        phys::Vec2 b(3.0f, 2.0f);
        a - b;
        REQUIRE(a.x == 5.0f);
        REQUIRE(a.y == 6.0f);
        REQUIRE(b.x == 3.0f);
        REQUIRE(b.y == 2.0f);
    }

    SECTION("length of zero vector")
    {
        phys::Vec2 a{};
        REQUIRE(a.length() == 0.0f);
    }

    SECTION("length 3-4-5 triangle")
    {
        phys::Vec2 a(3.0f, 4.0f);
        REQUIRE(a.length() == Catch::Approx(5.0f));
    }

    SECTION("length of unit vector")
    {
        phys::Vec2 a(1.0f, 0.0f);
        REQUIRE(a.length() == Catch::Approx(1.0f));
    }

    SECTION("length with negative components")
    {
        phys::Vec2 a(-3.0f, -4.0f);
        REQUIRE(a.length() == Catch::Approx(5.0f));
    }

    SECTION("normalize produces unit length")
    {
        phys::Vec2 a(3.0f, 4.0f);
        a.normalize();
        REQUIRE(a.length() == Catch::Approx(1.0f));
    }

    SECTION("normalize preserves direction")
    {
        phys::Vec2 a(3.0f, 4.0f);
        a.normalize();
        REQUIRE(a.x == Catch::Approx(0.6f));
        REQUIRE(a.y == Catch::Approx(0.8f));
    }

    SECTION("normalize already unit vector")
    {
        phys::Vec2 a(1.0f, 0.0f);
        a.normalize();
        REQUIRE(a.x == Catch::Approx(1.0f));
        REQUIRE(a.y == Catch::Approx(0.0f));
    }

    SECTION("normalize with negative components")
    {
        phys::Vec2 a(-3.0f, -4.0f);
        a.normalize();
        REQUIRE(a.length() == Catch::Approx(1.0f));
        REQUIRE(a.x == Catch::Approx(-0.6f));
        REQUIRE(a.y == Catch::Approx(-0.8f));
    }

    SECTION("dot product basic")
    {
        phys::Vec2 a(1.0f, 2.0f);
        phys::Vec2 b(3.0f, 4.0f);
        REQUIRE((a * b) == Catch::Approx(11.0f));
    }

    SECTION("dot product is commutative")
    {
        phys::Vec2 a(1.0f, 2.0f);
        phys::Vec2 b(3.0f, 4.0f);
        REQUIRE((a * b) == Catch::Approx(b * a));
    }

    SECTION("dot product of perpendicular vectors is zero")
    {
        phys::Vec2 a(1.0f, 0.0f);
        phys::Vec2 b(0.0f, 1.0f);
        REQUIRE((a * b) == Catch::Approx(0.0f));
    }

    SECTION("dot product with zero vector is zero")
    {
        phys::Vec2 a(3.0f, 4.0f);
        phys::Vec2 zero{};
        REQUIRE((a * zero) == Catch::Approx(0.0f));
    }

    SECTION("dot product of vector with itself equals length squared")
    {
        phys::Vec2 a(3.0f, 4.0f);
        REQUIRE((a * a) == Catch::Approx(a.length() * a.length()));
    }

}
