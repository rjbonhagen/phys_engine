# phys_engine

[![CI](https://github.com/rjbonhagen/phys_engine/actions/workflows/ci.yml/badge.svg)](https://github.com/rjbonhagen/phys_engine/actions/workflows/ci.yml)

A small 2D physics engine written in C++23, with an SDL2 + Dear ImGui sandbox for
experimenting with circle and box collisions in real time.

## Features

- Symplectic Euler integration (`a = F/m`, `v += a·dt`, `x += v·dt`)
- Circle vs. circle and circle/box vs. box collision detection
- Impulse-based collision response with configurable restitution
- Resizable world bounds with static box walls
- Interactive sandbox: spawn circles/boxs, select, drag, and delete objects via an
  ImGui control panel

## Performance

Uniform spatial hash broad phase versus the all-pairs reference, measured with
the headless `bench` target (Release, 120 steps per case, medians, constant body
density). Bodies are 0.25-unit circles under gravity in a walled arena.

| bodies | all-pairs step | all-pairs pairs tested | spatial hash step | pairs tested | speedup |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 100 | 0.081 ms | 5,356 | 0.044 ms | 138 | 1.9x |
| 500 | 1.873 ms | 126,756 | 0.257 ms | 424 | 7.3x |
| 1,000 | 7.326 ms | 503,506 | 0.485 ms | 693 | 15.1x |
| 2,000 | 30.297 ms | 2,007,006 | 0.885 ms | 1,236 | 34.2x |
| 5,000 | 189.119 ms | 12,517,506 | 2.185 ms | 2,739 | 86.6x |
| 10,000 | not measured | — | 4.736 ms | 5,270 | — |
| 20,000 | not measured | — | 10.694 ms | 10,100 | — |

All-pairs is not measured past 5,000 bodies: at 20,000 it is 200 million pair
tests per step.

At a 16.67 ms budget for 60 Hz, all-pairs runs out between 1,000 and 2,000
bodies. The spatial hash still has headroom at 20,000, where the physics step
alone could sustain 94 Hz. These figures include the swept-collision pass added
afterwards, which cost about 5 percent of the speedup at 5,000 bodies: it was
91.5x before continuous detection was added, and 86.6x after. Pair tests drop from 12.5 million to 2,739 at 5,000
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
  - `Box.hpp` — oriented box shape
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

- Switch between **Select / Move**, **Add Circle**, and **Add box** modes
- Click in the world to spawn an object (in an "Add" mode) or select/drag an existing
  one (in Select mode)
- Press **Delete** (or use the panel button) to remove the selected object
- Resize the world bounds with the Width/Height sliders
- Browse and select objects from the object list

## API notes

A few things that are easy to get wrong:

- `Scene::add_circle(position, velocity, acceleration, radius, forces, mass, restitution)`
  takes **radius before forces**. Both `add_circle` and `add_aabb` throw
  `std::invalid_argument` on a non-positive mass; `Box`'s constructor throws if
  `min == max`.
- An `Box` is stored as a centre (`Object::position`) plus a half extent. Use
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
- Fast bodies are swept against static geometry before detection runs, and a
  body is stopped at its first time of impact. There is no speed cap: a circle
  at 5000 units/s is held by a 0.2-unit-thick wall.

  **What this does not cover.** Only *static* bodies are swept, and only circles
  are swept against them. Two fast dynamic bodies can still pass through each
  other, a fast box is not swept at all, and the box sweep treats the grown
  box's corners as square rather than rounded, so it stops a body marginally
  early near one. Circles are also still clamped to the scene bounds by
  `resolve_border_collision_circle`, which remains the only thing containing a
  scene built without walls -- that is a bounds policy, not a tunnelling
  backstop, and it has not been retired.
- Contact impulses are applied **at the contact point**, so they produce torque
  as well as force. A disc on a ramp rolls: with enough friction it accelerates
  at `g*sin(theta) / 1.5` rather than `g*sin(theta)`, because a solid disc has
  `I = m r^2 / 2` and a third of the work goes into spin. Measured 3.267 against
  a predicted 3.267 on a 30 degree slope, with the contact point exactly
  stationary.

  **Boxes rotate too.** `phys::Box` is an oriented box: centre, half extent and
  an orientation. Collision is a separating-axis test over the four face normals
  with reference-face clipping, giving up to two contact points. Two matters: a
  single point lets a box resting flat pivot about it and tip for no reason.
  Rectangle inertia is `m (w^2 + h^2) / 12`.

  A nudged tall box topples through a quarter turn and comes to rest on its
  side. A box lying flat on a slope obeys the Coulomb criterion exactly: held at
  mu = 0.8 on 30 degrees where tan is 0.577, sliding at mu = 0.2, and sliding on
  50 degrees where tan is 1.192 whatever the friction.

  **`Box::get_min()` and `get_max()` are the world-space *bounding* box**, which
  grows as the box rotates. They coincide with the box itself only at
  orientation zero. Use `corners()` for the actual shape and `get_half_body()`
  for the local extent.

  **No rolling resistance is modelled**, so a disc that reaches rolling keeps
  rolling indefinitely on level ground. A box balanced on a corner on a slope
  tumbles rather than settling, which is a real unstable equilibrium rather than
  a bug.

- **Distance joints** hold two anchors a fixed distance apart, with the second
  anchor optionally a fixed world point, which is what a pendulum pivot is.
  Anchors are stored in each body's local frame so they rotate with it, and an
  off-centre anchor therefore applies torque. Solved in the same iteration loop
  as contacts, with Baumgarte bias so length error is corrected rather than
  accumulated.

  A pendulum released horizontally measures a 3.352 s period against 3.350 s
  predicted for a 90 degree amplitude, and a five-link chain holds every gap to
  within 0.0002. Rest length defaults to the anchor separation at creation;
  pass one explicitly for a rope or rod of a chosen length.

- **Moving a body by hand needs `Scene::wake`.** A sleeping body skips
  integration, so dragging a support out from under a settled stack would leave
  it hanging in the air. `wake` rouses the body, its resting island, and
  anything it was touching. `remove_object` calls it too. The sandbox calls it
  on every drag.

- `phys::real` is a `float` typedef. All math is written against it so precision
  can be changed in one place.
