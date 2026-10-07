#pragma once
#include "phys/math/Real.hpp"

// Turns variable frame times into a whole number of fixed physics steps, so
// behaviour does not depend on refresh rate. No SDL dependency, so it is
// directly testable.
class StepClock
{
    public:
    // Time beyond max_frame is discarded; max_steps caps the steps one call can
    // return, which is what prevents catch-up work from spiralling.
    explicit StepClock(phys::real step      = 1.0f / 120.0f,
                       phys::real max_frame = 0.25f,
                       int        max_steps = 8)
        : STEP(step), MAX_FRAME(max_frame), MAX_STEPS(max_steps) {}

    phys::real step_size() const { return STEP; }
    phys::real max_frame() const { return MAX_FRAME; }
    int        max_steps() const { return MAX_STEPS; }

    // Unsimulatable time is discarded rather than banked: the accumulator is
    // always left below one step, so no call inherits a backlog. The simulation
    // falls behind the wall clock by dropped_time() instead of skipping ahead.
    int advance(phys::real elapsed)
    {
        if (!(elapsed > 0.0f)) elapsed = 0.0f;   // also rejects NaN

        if (elapsed > MAX_FRAME)
        {
            dropped += static_cast<double>(elapsed - MAX_FRAME);
            elapsed  = MAX_FRAME;
        }

        accumulated += static_cast<double>(elapsed);

        const double step = static_cast<double>(STEP);
        int steps = static_cast<int>(accumulated / step);

        if (steps > MAX_STEPS)
        {
            const int discarded = steps - MAX_STEPS;
            dropped     += discarded * step;
            accumulated -= discarded * step;
            steps        = MAX_STEPS;
        }

        accumulated -= steps * step;
        return steps;
    }

    phys::real dropped_time() const { return static_cast<phys::real>(dropped); }
    phys::real accumulator() const { return static_cast<phys::real>(accumulated); }

    // Call when resuming from a pause so the paused time is not simulated.
    void reset() { accumulated = 0.0; }

    private:
    phys::real STEP;
    phys::real MAX_FRAME;
    int        MAX_STEPS;

    // Double, not phys::real: summed every frame for the life of the session.
    double accumulated{0.0};
    double dropped{0.0};
};
