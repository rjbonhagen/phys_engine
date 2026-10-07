#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "StepClock.hpp"

// Feeds `seconds` of real time to the clock in frames of `frame` length and
// returns the total number of fixed steps it asked for.
static int simulate(StepClock& clock, phys::real seconds, phys::real frame)
{
    int total  = 0;
    int frames = static_cast<int>(seconds / frame + 0.5f);
    for (int i = 0; i < frames; i++) total += clock.advance(frame);
    return total;
}

TEST_CASE("StepClock decouples physics from frame rate")
{
    SECTION("30, 60 and 144 FPS schedules yield the same step count")
    {
        // One second at a 120 Hz fixed step is 120 steps, whatever the frame
        // schedule. One step of slack: a frame length like 1.0f/144.0f is not
        // exactly representable, so a nominal second of such frames can sum to
        // a hair under one second and come up one step short. The clock is
        // right to decline a step it was not given time for -- what matters is
        // that the shortfall stays bounded and does not grow with frame rate.
        StepClock a, b, c;
        const int at30  = simulate(a, 1.0f, 1.0f / 30.0f);
        const int at60  = simulate(b, 1.0f, 1.0f / 60.0f);
        const int at144 = simulate(c, 1.0f, 1.0f / 144.0f);

        REQUIRE(at30 >= 119);
        REQUIRE(at30 <= 120);
        REQUIRE(at60 >= 119);
        REQUIRE(at60 <= 120);
        REQUIRE(at144 >= 119);
        REQUIRE(at144 <= 120);
    }

    SECTION("equal step counts give identical simulation time")
    {
        // The guarantee the engine actually relies on: every step is the same
        // size, so N steps is always the same amount of simulated time no
        // matter how the frames were shaped.
        StepClock fast, slow;
        int fast_steps = 0, slow_steps = 0;
        for (int i = 0; i < 600; i++) fast_steps += fast.advance(1.0f / 300.0f);
        for (int i = 0; i < 100; i++) slow_steps += slow.advance(1.0f / 50.0f);

        REQUIRE(fast_steps * fast.step_size()
                == Catch::Approx(slow_steps * slow.step_size()));
    }

    SECTION("a frame shorter than one step produces no step, then catches up")
    {
        StepClock clock;                        // step = 1/120
        REQUIRE(clock.advance(1.0f / 240.0f) == 0);   // half a step
        REQUIRE(clock.advance(1.0f / 240.0f) == 1);   // the other half
    }

    SECTION("the remainder is always left below one step")
    {
        StepClock clock;
        for (phys::real frame : {0.001f, 0.007f, 1.0f / 60.0f, 0.033f, 0.1f})
        {
            clock.advance(frame);
            REQUIRE(clock.accumulator() < clock.step_size());
            REQUIRE(clock.accumulator() >= 0.0f);
        }
    }

    SECTION("exact multiples of the step leave no remainder")
    {
        StepClock clock;
        REQUIRE(clock.advance(4.0f / 120.0f) == 4);
        REQUIRE(clock.accumulator() == Catch::Approx(0.0f).margin(1e-6f));
    }
}

TEST_CASE("StepClock bounds catch-up work")
{
    SECTION("a long stall cannot produce an unbounded burst of steps")
    {
        StepClock clock;                        // max_frame 0.25 s, max_steps 8
        const int steps = clock.advance(10.0f); // a ten second stall

        REQUIRE(steps <= clock.max_steps());
        REQUIRE(clock.dropped_time() > 0.0f);
    }

    SECTION("time beyond max_frame is dropped, not banked")
    {
        StepClock clock;
        clock.advance(10.0f);

        // The next frame must start from a near-empty accumulator, so the stall
        // cannot keep generating catch-up work on later frames.
        REQUIRE(clock.accumulator() < clock.step_size());
        REQUIRE(clock.advance(1.0f / 60.0f) <= 2);
    }

    SECTION("dropped time accumulates across stalls")
    {
        StepClock clock;
        clock.advance(5.0f);
        const phys::real after_first = clock.dropped_time();
        clock.advance(5.0f);

        REQUIRE(clock.dropped_time() > after_first);
    }

    SECTION("a steady frame rate drops nothing")
    {
        StepClock clock;
        simulate(clock, 5.0f, 1.0f / 60.0f);

        REQUIRE(clock.dropped_time() == Catch::Approx(0.0f).margin(1e-6f));
    }

    SECTION("the catch-up cap is what bounds a stall, not max_frame alone")
    {
        // max_frame 0.25 s at a 1/120 step is 30 steps' worth, so the 8-step
        // cap is the binding constraint and the rest is discarded.
        StepClock clock{1.0f / 120.0f, 0.25f, 8};
        REQUIRE(clock.advance(0.25f) == 8);
        REQUIRE(clock.dropped_time() > 0.0f);
    }
}

TEST_CASE("StepClock handles degenerate input")
{
    SECTION("zero and negative elapsed times are ignored")
    {
        StepClock clock;
        REQUIRE(clock.advance(0.0f) == 0);
        REQUIRE(clock.advance(-1.0f) == 0);
        REQUIRE(clock.accumulator() == Catch::Approx(0.0f).margin(1e-6f));
        REQUIRE(clock.dropped_time() == Catch::Approx(0.0f).margin(1e-6f));
    }

    SECTION("reset clears the pending remainder but keeps dropped time")
    {
        StepClock clock;
        clock.advance(10.0f);
        const phys::real dropped = clock.dropped_time();

        clock.advance(1.0f / 240.0f);   // bank half a step
        clock.reset();

        REQUIRE(clock.accumulator() == Catch::Approx(0.0f).margin(1e-6f));
        REQUIRE(clock.dropped_time() == Catch::Approx(dropped));
    }

    SECTION("a custom step size is honoured")
    {
        StepClock clock{1.0f / 60.0f};
        REQUIRE(clock.step_size() == Catch::Approx(1.0f / 60.0f));
        REQUIRE(clock.advance(1.0f / 60.0f) == 1);
    }
}
