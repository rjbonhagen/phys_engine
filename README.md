# phys_engine

[![CI](https://github.com/rjbonhagen/phys_engine/actions/workflows/ci.yml/badge.svg)](https://github.com/rjbonhagen/phys_engine/actions/workflows/ci.yml)

A small 2D physics engine written in C++23, with an SDL2 + Dear ImGui sandbox for
experimenting with circle and AABB collisions in real time.

## Features

- Symplectic Euler integration (`a = F/m`, `v += a·dt`, `x += v·dt`)
- Circle vs. circle and circle/AABB vs. AABB collision detection
- Impulse-based collision response with configurable restitution
- Resizable world bounds with static AABB walls
- Interactive sandbox: spawn circles/AABBs, select, drag, and delete objects via an
  ImGui control panel

## Performance

Uniform spatial hash broad phase versus the all-pairs reference, measured with
the headless `bench` target (Release, 120 steps per case, medians, constant body
density). Bodies are 0.25-unit circles under gravity in a walled arena.

| bodies | all-pairs step | all-pairs pairs tested | spatial hash step | pairs tested | speedup |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 100 | 0.085 ms | 5,356 | 0.042 ms | 138 | 2.1x |
| 500 | 1.878 ms | 126,756 | 0.251 ms | 424 | 7.5x |
| 1,000 | 7.349 ms | 503,506 | 0.455 ms | 693 | 16.2x |
| 2,000 | 29.910 ms | 2,007,006 | 0.835 ms | 1,236 | 35.8x |
| 5,000 | 191.060 ms | 12,517,506 | 2.087 ms | 2,739 | 91.5x |
| 10,000 | not measured | — | 4.582 ms | 5,270 | — |
| 20,000 | not measured | — | 10.297 ms | 10,100 | — |

All-pairs is not measured past 5,000 bodies: at 20,000 it is 200 million pair
tests per step.

At a 16.67 ms budget for 60 Hz, all-pairs runs out between 1,000 and 2,000
bodies. The spatial hash still has headroom at 20,000, where the physics step
alone could sustain 97 Hz. Pair tests drop from 12.5 million to 2,739 at 5,000
bodies, a factor of 4,570, and grow linearly with body count rather than
quadratically.

The two broad phases are checked against each other: a test runs the same
600-step scene through both and asserts the contact count matches on every
step, so the grid cannot quietly miss a pair. All-pairs is kept as that
reference and is selectable via `Scene::set_broad_phase`.

Caveats worth stating. These are single-machine numbers, not a claim about
hardware in general. Density is held constant as body count grows, so these
measure the broad phase rather than contact resolution; a dense pile would shift
the cost into the solver. Planes are infinite half-spaces and cannot be
bucketed, so each one is paired against every body.

Run it yourself:

```
cmake --build build/windows-x64-debug --config Release --target bench
./build/windows-x64-debug/Release/bench.exe 120
```

## Project layout

- `include/phys/` — header-only physics library
  - `math/Real.hpp` — `phys::real` typedef (all math is written against this)
  - `math/Vec2.hpp` — 2D vector math
  - `Object.hpp` — base physics object (position, velocity, acceleration, forces, mass, restitution)
  - `Circle.hpp` — circle shape
  - `AABB.hpp` — axis-aligned bounding box shape
  - `Manifold.hpp` — collision manifold (colliding pair, normal, penetration, contact point)
- `include/Scene.hpp` / `src/Scene.cpp` — owns all objects, steps the simulation, resolves
  border and object collisions. No SDL dependency, so it's shared by both the game and the tests.
- `include/Renderer.hpp` / `src/Renderer.cpp` — SDL2 drawing
- `src/main.cpp` — the sandbox application (SDL2 event loop + ImGui UI)
- `tests/` — Catch2 unit and integration tests

## Building

### Windows (Visual Studio 2026, x64)

```powershell
cmake --preset "windows-x64-debug"
cmake --build build/windows-x64-debug --config Debug
```

Run the sandbox:

```powershell
.\build\windows-x64-debug\Debug\Game.exe
```

Run the tests:

```powershell
cd build/windows-x64-debug && ctest -C Debug
```

### macOS (Clang, arm64)

```bash
cmake --preset "Clang 17.0.0 arm64-apple-darwin25.1.0"
cmake --build "build/Clang 17.0.0 arm64-apple-darwin25.1.0"
```

Run the sandbox:

```bash
"build/Clang 17.0.0 arm64-apple-darwin25.1.0/Game"
```

Run the tests:

```bash
cd "build/Clang 17.0.0 arm64-apple-darwin25.1.0" && ctest
```

SDL2, Dear ImGui, and Catch2 are fetched automatically via CMake `FetchContent` at
configure time — no manual dependency installation needed.

## Using the sandbox

The "Controls" panel (top-left) lets you:

- Switch between **Select / Move**, **Add Circle**, and **Add AABB** modes
- Click in the world to spawn an object (in an "Add" mode) or select/drag an existing
  one (in Select mode)
- Press **Delete** (or use the panel button) to remove the selected object
- Resize the world bounds with the Width/Height sliders
- Browse and select objects from the object list

## API notes

A few things that are easy to get wrong:

- `Scene::add_circle(position, velocity, acceleration, radius, forces, mass, restitution)`
  takes **radius before forces**. Both `add_circle` and `add_aabb` throw
  `std::invalid_argument` on a non-positive mass; `AABB`'s constructor throws if
  `min == max`.
- An `AABB` is stored as a centre (`Object::position`) plus a half extent. Use
  `get_min()` / `get_max()` to read its corners -- they are derived, so they stay
  correct as the box moves. `resize()` changes the extent.
- `Scene::step(dt)` runs four passes in order: integrate every body (symplectic
  Euler), clamp circles to the scene bounds, detect overlapping pairs into
  manifolds, then resolve them.
- Collision response uses `j = -(1 + e) * v_rel·n / (1/mA + 1/mB)`, where `e` is
  the **product** of the two restitutions. That means a single body with
  restitution 1.0 is enough to make a pair perfectly elastic, and a scene where
  everything is 1.0 never dissipates energy -- bodies will not settle.
- Infinite mass (`INFINITY`) marks a body as static: it contributes zero inverse
  mass, so it absorbs no impulse and is never repositioned.
- `Scene` clamps speed to `MAX_SPEED` (50 units/s) inside `integrate`. This is a
  tunnelling backstop sized against the 1-unit wall thickness; thinner colliders
  would need a lower ceiling or substepping.
- `phys::real` is a `float` typedef. All math is written against it so precision
  can be changed in one place.
