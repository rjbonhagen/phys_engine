// Headless benchmark. No SDL, no rendering: measures the simulation step only.
// Scenes are generated from a fixed seed so runs are comparable across builds.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>

#include "Scene.hpp"

using namespace phys;

static constexpr real DT = 1.0f / 120.0f;

struct Result
{
    double step_ms;
    double detect_ms;
    double solve_ms;
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
// as the count grows, so a larger run means a larger arena rather than a denser
// pile -- otherwise the contact count, not the broad phase, would dominate.
static Result run(size_t bodies, int steps, Scene::BroadPhase bp)
{
    const real side  = std::sqrt(static_cast<real>(bodies)) * 1.6f;
    const real width = std::max(side, 8.0f);

    Scene scene{width, width};
    scene.set_broad_phase(bp);
    scene.create_walls(1.0f);

    std::mt19937 rng{1234567u};
    std::uniform_real_distribution<float> pos(1.0f, width - 1.0f);
    std::uniform_real_distribution<float> vel(-3.0f, 3.0f);

    for (size_t i = 0; i < bodies; i++)
        scene.add_circle({pos(rng), pos(rng)}, {vel(rng), vel(rng)},
                         Vec2{}, 0.25f, Vec2{0.0f, -9.8f}, 1.0f, 0.4f);

    std::vector<double> step_ms, detect_ms, solve_ms;
    step_ms.reserve(steps);
    detect_ms.reserve(steps);
    solve_ms.reserve(steps);

    double pairs = 0.0, contacts = 0.0;

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
    const int steps = (argc > 1) ? std::atoi(argv[1]) : 120;

    std::printf("phys_engine benchmark, %d steps per case, medians\n\n", steps);

    struct Row { size_t n; Result all; Result hash; };
    std::vector<Row> rows;

    // All-pairs is skipped past 5000: at 20000 bodies that is 200 million pair
    // tests per step, which would dominate the whole run and tell us nothing.
    constexpr size_t ALL_PAIRS_LIMIT = 5000;

    for (size_t n : {100u, 500u, 1000u, 2000u, 5000u, 10000u, 20000u})
        rows.push_back({n,
                        (n <= ALL_PAIRS_LIMIT) ? run(n, steps, Scene::BroadPhase::AllPairs)
                                               : Result{},
                        run(n, steps, Scene::BroadPhase::SpatialHash)});

    std::printf("%7s |      all-pairs      |     spatial hash    |\n", "");
    std::printf("%7s | %9s %10s | %9s %10s | %8s\n",
                "bodies", "step ms", "pairs", "step ms", "pairs", "speedup");
    std::printf("--------+---------------------+---------------------+---------\n");

    for (const Row& r : rows)
    {
        if (r.all.step_ms == 0.0)
        {
            std::printf("%7zu | %9s %10s | %9.3f %10.0f | %8s\n",
                        r.n, "skipped", "-", r.hash.step_ms, r.hash.candidate_pairs, "-");
            continue;
        }

        const double speedup = (r.hash.step_ms > 0.0) ? r.all.step_ms / r.hash.step_ms : 0.0;
        std::printf("%7zu | %9.3f %10.0f | %9.3f %10.0f | %7.1fx\n",
                    r.n, r.all.step_ms, r.all.candidate_pairs,
                    r.hash.step_ms, r.hash.candidate_pairs, speedup);
    }

    bool same = true;
    for (const Row& r : rows)
        if (r.all.step_ms > 0.0 && r.all.contacts != r.hash.contacts) same = false;
    std::printf("\ncontacts identical between the two broad phases: %s\n",
                same ? "yes" : "NO - the grid is missing pairs");

    std::printf("\nspatial hash detail (fps cap is 1000 / step ms, physics only):\n");
    std::printf("%7s %11s %11s %10s %9s\n", "bodies", "detect ms", "solve ms", "contacts", "fps cap");
    for (const Row& r : rows)
        std::printf("%7zu %11.3f %11.3f %10.0f %9.0f\n", r.n,
                    r.hash.detect_ms, r.hash.solve_ms, r.hash.contacts,
                    r.hash.step_ms > 0.0 ? 1000.0 / r.hash.step_ms : 0.0);

    return 0;
}
