// Headless benchmark. No SDL, no rendering: measures the simulation step only.
// Scenes are generated from a fixed seed so runs are comparable across builds.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include "Scene.hpp"

using namespace phys;

static constexpr real DT = 1.0f / 120.0f;

struct Result
{
    double step_ms_median;
    double detect_ms_median;
    double solve_ms_median;
    double candidate_pairs;
    double contacts;
};

static double median(std::vector<double>& v)
{
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}

// A box of bodies dropped into a walled arena. Density is held roughly constant
// as the count grows, so larger runs mean a larger arena rather than a denser
// pile -- otherwise the contact count, not the broad phase, would dominate.
static Result run(size_t bodies, int steps)
{
    const real side  = std::sqrt(static_cast<real>(bodies)) * 1.6f;
    const real width = std::max(side, 8.0f);

    Scene scene{width, width};
    scene.create_walls(1.0f);

    std::mt19937 rng{1234567u};
    std::uniform_real_distribution<float> pos(1.0f, width - 1.0f);
    std::uniform_real_distribution<float> vel(-3.0f, 3.0f);

    for (size_t i = 0; i < bodies; i++)
    {
        const Vec2 p{pos(rng), pos(rng)};
        const Vec2 v{vel(rng), vel(rng)};
        scene.add_circle(p, v, Vec2{}, 0.25f, Vec2{0.0f, -9.8f}, 1.0f, 0.4f);
    }

    std::vector<double> step_ms, detect_ms, solve_ms;
    double pairs = 0.0, contacts = 0.0;

    step_ms.reserve(steps);
    detect_ms.reserve(steps);
    solve_ms.reserve(steps);

    for (int i = 0; i < steps; i++)
    {
        scene.step(DT);
        const auto& s = scene.get_stats();

        step_ms.push_back(s.step_ms);
        detect_ms.push_back(s.detect_ms);
        solve_ms.push_back(s.solve_ms);
        pairs    += static_cast<double>(s.candidate_pairs);
        contacts += static_cast<double>(s.contacts);
    }

    return {median(step_ms), median(detect_ms), median(solve_ms),
            pairs / steps, contacts / steps};
}

int main(int argc, char** argv)
{
    int steps = 300;
    if (argc > 1) steps = std::atoi(argv[1]);

    std::printf("phys_engine benchmark, %d steps per case, medians\n\n", steps);
    std::printf("%7s %11s %11s %11s %13s %10s %9s\n",
                "bodies", "step ms", "detect ms", "solve ms", "cand pairs", "contacts", "fps cap");
    std::printf("%7s %11s %11s %11s %13s %10s %9s\n",
                "------", "-------", "---------", "--------", "----------", "--------", "-------");

    for (size_t n : {100u, 500u, 1000u, 2000u, 5000u})
    {
        const Result r = run(n, steps);
        const double fps = (r.step_ms_median > 0.0) ? 1000.0 / r.step_ms_median : 0.0;

        std::printf("%7zu %11.3f %11.3f %11.3f %13.0f %10.0f %9.0f\n",
                    n, r.step_ms_median, r.detect_ms_median, r.solve_ms_median,
                    r.candidate_pairs, r.contacts, fps);
    }

    std::printf("\nfps cap is 1000 / step ms: the rate the physics alone could sustain.\n");
    return 0;
}
