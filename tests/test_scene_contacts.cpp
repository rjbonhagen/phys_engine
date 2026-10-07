#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>
#include <stdexcept>

#include "Scene.hpp"
#include "phys/Box.hpp"
#include "phys/Circle.hpp"

static phys::Circle& as_circle(phys::Object& o) { return static_cast<phys::Circle&>(o); }

static const phys::Vec2 ZERO{0.0f, 0.0f};

TEST_CASE("Scene Box-Box collision")
{
    SECTION("boxes approaching along x are separated along x")
    {
        // The pair overlaps more on y than on x, so x is the axis of least
        // penetration and the response should act along it.
        Scene scene{40.0f, 40.0f};
        scene.add_box({8.0f, 10.0f}, {12.0f, 20.0f}, {10.0f, 0.0f}, ZERO, ZERO, 1.0f, 0.5f);
        scene.add_box({11.0f, 10.0f}, {15.0f, 20.0f}, {-10.0f, 0.0f}, ZERO, ZERO, 1.0f, 0.5f);

        scene.step(1.0f / 60.0f);

        REQUIRE(scene.get_objects()[0]->velocity.x < 0.0f);
        REQUIRE(scene.get_objects()[1]->velocity.x > 0.0f);
        REQUIRE(scene.get_objects()[0]->velocity.y == Catch::Approx(0.0f));
    }

    SECTION("boxes approaching along y are separated along y")
    {
        Scene scene{40.0f, 40.0f};
        scene.add_box({10.0f, 8.0f}, {20.0f, 12.0f}, {0.0f, 10.0f}, ZERO, ZERO, 1.0f, 0.5f);
        scene.add_box({10.0f, 11.0f}, {20.0f, 15.0f}, {0.0f, -10.0f}, ZERO, ZERO, 1.0f, 0.5f);

        scene.step(1.0f / 60.0f);

        REQUIRE(scene.get_objects()[0]->velocity.y < 0.0f);
        REQUIRE(scene.get_objects()[1]->velocity.y > 0.0f);
        REQUIRE(scene.get_objects()[0]->velocity.x == Catch::Approx(0.0f));
    }

    SECTION("separated boxes are left alone")
    {
        Scene scene{40.0f, 40.0f};
        scene.add_box({2.0f, 2.0f}, {4.0f, 4.0f}, ZERO, ZERO, ZERO, 1.0f, 0.5f);
        scene.add_box({20.0f, 20.0f}, {22.0f, 22.0f}, ZERO, ZERO, ZERO, 1.0f, 0.5f);

        const phys::Vec2 a = scene.get_objects()[0]->position;
        const phys::Vec2 b = scene.get_objects()[1]->position;

        scene.step(1.0f / 60.0f);

        REQUIRE(scene.get_objects()[0]->position == a);
        REQUIRE(scene.get_objects()[1]->position == b);
    }
}

TEST_CASE("Scene collision response conserves momentum")
{
    SECTION("equal masses, head-on")
    {
        Scene scene{400.0f, 400.0f};
        scene.add_circle({195.0f, 200.0f}, {10.0f, 0.0f}, ZERO, 5.0f, ZERO, 1.0f, 1.0f);
        scene.add_circle({205.0f, 200.0f}, {-4.0f, 0.0f}, ZERO, 5.0f, ZERO, 1.0f, 1.0f);

        auto& o = scene.get_objects();
        const phys::real before = o[0]->mass * o[0]->velocity.x + o[1]->mass * o[1]->velocity.x;

        scene.step(0.01f);

        const phys::real after = o[0]->mass * o[0]->velocity.x + o[1]->mass * o[1]->velocity.x;
        REQUIRE(after == Catch::Approx(before).margin(0.001f));
    }

    SECTION("unequal masses, head-on")
    {
        // A 10:1 ratio. Momentum is still conserved, and the light body takes
        // the larger share of the velocity change.
        Scene scene{400.0f, 400.0f};
        scene.add_circle({195.0f, 200.0f}, {10.0f, 0.0f}, ZERO, 5.0f, ZERO, 10.0f, 1.0f);
        scene.add_circle({205.0f, 200.0f}, {-10.0f, 0.0f}, ZERO, 5.0f, ZERO, 1.0f, 1.0f);

        auto& o = scene.get_objects();
        const phys::real heavy_before = o[0]->velocity.x;
        const phys::real light_before = o[1]->velocity.x;
        const phys::real before = 10.0f * heavy_before + 1.0f * light_before;

        scene.step(0.01f);

        const phys::real after = 10.0f * o[0]->velocity.x + 1.0f * o[1]->velocity.x;
        REQUIRE(after == Catch::Approx(before).margin(0.001f));

        REQUIRE(std::fabs(o[1]->velocity.x - light_before)
                > std::fabs(o[0]->velocity.x - heavy_before));
    }

    SECTION("a fully inelastic collision loses kinetic energy")
    {
        Scene scene{400.0f, 400.0f};
        scene.add_circle({195.0f, 200.0f}, {10.0f, 0.0f}, ZERO, 5.0f, ZERO, 1.0f, 0.0f);
        scene.add_circle({205.0f, 200.0f}, {-10.0f, 0.0f}, ZERO, 5.0f, ZERO, 1.0f, 0.0f);

        auto& o = scene.get_objects();
        auto kinetic_energy = [&o] {
            phys::real k = 0.0f;
            for (const auto& b : o)
            {
                const phys::real speed = b->velocity.length();
                k += 0.5f * b->mass * speed * speed;
            }
            return k;
        };

        const phys::real before = kinetic_energy();
        scene.step(0.01f);

        REQUIRE(kinetic_energy() < before);
    }
}

TEST_CASE("Scene object management")
{
    SECTION("remove_object erases the object at the index")
    {
        Scene scene{100.0f, 100.0f};
        scene.add_circle({10.0f, 10.0f}, ZERO, ZERO, 1.0f, ZERO, 1.0f, 0.5f);
        scene.add_circle({50.0f, 50.0f}, ZERO, ZERO, 2.0f, ZERO, 1.0f, 0.5f);

        scene.remove_object(0);

        REQUIRE(scene.get_objects().size() == 1);
        REQUIRE(as_circle(*scene.get_objects()[0]).radius == 2.0f);
    }

    SECTION("remove_object ignores an out-of-range index")
    {
        Scene scene{100.0f, 100.0f};
        scene.add_circle({10.0f, 10.0f}, ZERO, ZERO, 1.0f, ZERO, 1.0f, 0.5f);

        scene.remove_object(99);

        REQUIRE(scene.get_objects().size() == 1);
    }

    SECTION("non-positive mass is rejected")
    {
        Scene scene{100.0f, 100.0f};

        REQUIRE_THROWS_AS(scene.add_circle({1.0f, 1.0f}, ZERO, ZERO, 1.0f, ZERO, 0.0f, 0.5f),
                          std::invalid_argument);
        REQUIRE_THROWS_AS(scene.add_box({0.0f, 0.0f}, {1.0f, 1.0f}, ZERO, ZERO, ZERO, -1.0f, 0.5f),
                          std::invalid_argument);
    }

    SECTION("create_walls encloses the scene and set_dimensions moves the walls")
    {
        Scene scene{10.0f, 8.0f};
        scene.create_walls(1.0f);

        REQUIRE(scene.get_objects().size() == 4);

        auto* bottom = static_cast<phys::Box*>(scene.get_objects()[0].get());
        auto* top    = static_cast<phys::Box*>(scene.get_objects()[1].get());
        auto* left   = static_cast<phys::Box*>(scene.get_objects()[2].get());
        auto* right  = static_cast<phys::Box*>(scene.get_objects()[3].get());

        REQUIRE(bottom->get_max().y == Catch::Approx(0.0f));
        REQUIRE(top->get_min().y == Catch::Approx(8.0f));
        REQUIRE(left->get_max().x == Catch::Approx(0.0f));
        REQUIRE(right->get_min().x == Catch::Approx(10.0f));

        scene.set_dimensions(20.0f, 16.0f);

        REQUIRE(right->get_min().x == Catch::Approx(20.0f));
        REQUIRE(top->get_min().y == Catch::Approx(16.0f));
    }

    SECTION("a walled scene keeps a bouncing circle inside")
    {
        Scene scene{10.0f, 8.0f};
        scene.create_walls(1.0f);
        scene.add_circle({5.0f, 6.0f}, {7.0f, 0.0f}, ZERO, 0.3f, {0.0f, -9.8f}, 1.0f, 0.6f);

        for (int i = 0; i < 60 * 20; i++) scene.step(1.0f / 60.0f);

        auto& c = as_circle(*scene.get_objects()[4]);
        REQUIRE(std::isfinite(c.velocity.length()));
        REQUIRE(c.position.x > 0.0f);
        REQUIRE(c.position.x < 10.0f);
        REQUIRE(c.position.y > 0.0f);
        REQUIRE(c.position.y < 8.0f);
    }
}

TEST_CASE("Box geometry")
{
    SECTION("corners are derived from the centre, so they follow a move")
    {
        phys::Box box{{0.0f, 0.0f}, {4.0f, 2.0f}, ZERO, ZERO, ZERO, 1.0f, 0.5f};

        REQUIRE(box.position == phys::Vec2{2.0f, 1.0f});
        REQUIRE(box.get_half_body() == phys::Vec2{2.0f, 1.0f});

        box.position += phys::Vec2{10.0f, 5.0f};

        REQUIRE(box.get_min() == phys::Vec2{10.0f, 5.0f});
        REQUIRE(box.get_max() == phys::Vec2{14.0f, 7.0f});
    }

    SECTION("resize sets both the extent and the centre")
    {
        phys::Box box{{0.0f, 0.0f}, {4.0f, 2.0f}, ZERO, ZERO, ZERO, 1.0f, 0.5f};

        box.resize({10.0f, 10.0f}, {12.0f, 20.0f});

        REQUIRE(box.get_min() == phys::Vec2{10.0f, 10.0f});
        REQUIRE(box.get_max() == phys::Vec2{12.0f, 20.0f});
        REQUIRE(box.position == phys::Vec2{11.0f, 15.0f});
    }

    SECTION("degenerate and inverted boxes are rejected")
    {
        REQUIRE_THROWS_AS((phys::Box{{1.0f, 1.0f}, {1.0f, 1.0f}, ZERO, ZERO, ZERO, 1.0f, 0.5f}),
                          std::invalid_argument);
        REQUIRE_THROWS_AS((phys::Box{{5.0f, 0.0f}, {1.0f, 2.0f}, ZERO, ZERO, ZERO, 1.0f, 0.5f}),
                          std::invalid_argument);
        REQUIRE_THROWS_AS((phys::Box{{0.0f, 5.0f}, {2.0f, 1.0f}, ZERO, ZERO, ZERO, 1.0f, 0.5f}),
                          std::invalid_argument);
    }
}
