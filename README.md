# phys_engine

[![CI](https://github.com/rjbonhagen/phys_engine/actions/workflows/ci.yml/badge.svg)](https://github.com/rjbonhagen/phys_engine/actions/workflows/ci.yml)

A 2D rigid-body physics engine in C++23, with an SDL2 and Dear ImGui sandbox for
inspecting it.

## Features

- Fixed 1/120 s timestep, decoupled from frame rate
- Circles, oriented boxes and static half-space planes
- Sequential impulse solver with warm starting and 8 iterations per step
- Coulomb friction, rotational dynamics, rolling
- Separating-axis test with reference-face clipping for box contacts
- Distance joints, body to body or body to a fixed world point
- Island sleeping with wake-on-contact
- Continuous collision detection against static geometry
- Uniform spatial hash broad phase, with all-pairs kept as a reference
- Resizable window; the world fits the window rather than tracking it

## Accuracy

Behaviour is tested against closed-form results rather than against recorded
output. From the test suite:

| case | measured | theory |
| --- | --- | --- |
| Disc rolling down a 30 degree ramp | 3.267 m/s² | `g·sin30 / 1.5` = 3.267 |
| Box sliding to rest, mu 0.3 | 24.54 m | `v²/(2·mu·g)` = 24.49 m |
| Pendulum period, released horizontal | 3.352 s | 3.350 s at 90 degree amplitude |
| Five-link hanging chain, link gaps | 1.0002, 1.0001, 1.0001, 1.0000 | 1.0 |
| Box on 30 degree slope, mu 0.8 | held, zero velocity | held, since mu > tan30 = 0.577 |
| Nudged tall box | topples to 90.2 degrees, then rests | quarter turn |

The rolling result is not special-cased. Friction acts at the contact point and
therefore applies torque; a solid disc has `I = m r² / 2`, so a third of the
work goes into spin and the acceleration is two thirds of the sliding value.
Measured contact slip is zero, which is the definition of rolling without
slipping.

## Performance

Spatial hash against the all-pairs reference. Headless `bench` target, Release,
120 steps per case, medians. Scene is 0.25-unit circles under gravity in a
walled arena, with body density held constant as the count grows.

| bodies | all-pairs | pairs tested | spatial hash | pairs tested | speedup |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 100 | 0.081 ms | 5,356 | 0.044 ms | 138 | 1.9x |
| 500 | 1.873 ms | 126,756 | 0.257 ms | 424 | 7.3x |
| 1,000 | 7.326 ms | 503,506 | 0.485 ms | 693 | 15.1x |
| 2,000 | 30.297 ms | 2,007,006 | 0.885 ms | 1,236 | 34.2x |
| 5,000 | 189.119 ms | 12,517,506 | 2.185 ms | 2,739 | 86.6x |
| 10,000 | not measured | | 4.736 ms | 5,270 | |
| 20,000 | not measured | | 10.694 ms | 10,100 | |

All-pairs is not measured past 5,000 bodies; at 20,000 it is 200 million pair
tests per step.

Against a 16.67 ms budget for 60 Hz, all-pairs runs out at roughly 1,500 bodies,
interpolating between the 1,000 and 2,000 rows. The spatial hash still has
headroom at 20,000, where the physics step alone could sustain 94 Hz. Pair tests
grow linearly with body count rather than quadratically.

Caveats: these are single-machine figures, and constant density means they
measure the broad phase rather than contact resolution. A dense pile moves the
cost into the solver, where the grid does not help.

A test runs the same 600-step scene through both broad phases and asserts the
contact count matches on every step, so the grid cannot silently drop a pair.
All-pairs remains selectable through `Scene::set_broad_phase`.

```
cmake --build build/windows-x64-debug --config Release --target bench
./build/windows-x64-debug/Release/bench.exe 120
```

## Building

SDL2, Dear ImGui and Catch2 are fetched by CMake at configure time. No manual
dependency installation.

Windows, Visual Studio, x64:

```powershell
cmake --preset "windows-x64-debug"
cmake --build build/windows-x64-debug --config Debug
.\build\windows-x64-debug\Debug\Game.exe
```

macOS, Clang, arm64:

```bash
cmake --preset "Clang 17.0.0 arm64-apple-darwin25.1.0"
cmake --build "build/Clang 17.0.0 arm64-apple-darwin25.1.0"
"build/Clang 17.0.0 arm64-apple-darwin25.1.0/Game"
```

Tests:

```powershell
cd build/windows-x64-debug && ctest -C Debug
```

CI builds and runs the suite on Windows, macOS and Linux. Use `--clean-first`
when checking for warnings locally; an incremental build can skip a translation
unit and hide warnings in it.

## Sandbox controls

The Controls panel is top left.

| control | effect |
| --- | --- |
| Select / Move | pick and drag a body, including walls and ramps |
| Add Circle, Add Box | spawn on click |
| Add Ramp | drag to set the slope, with a live preview |
| Add Joint | click two bodies to connect them, or a body then empty space to pin it |
| Delete Selected, Delete key | remove the selected body |
| Space | pause and resume |
| Period | advance one step while paused |
| Show contacts | draw contact points and normals, length scaled by penetration |
| Width, Height sliders | resize the world; walls follow |

The window is resizable. World size is a physics property driven by the
sliders, and the view scale is derived from both window and world size, so
resizing changes the view and never the simulation.

## Project layout

```
include/phys/           header-only physics core, no SDL dependency
  math/Real.hpp         phys::real typedef, currently float
  math/Vec2.hpp         vectors: dot, 2D cross, perpendicular
  Object.hpp            position, velocity, mass, restitution, friction, angular state
  Circle.hpp            circle, with solid-disc inertia
  Box.hpp               oriented box, with rectangle inertia
  Plane.hpp             static half-space
  Manifold.hpp          one contact: normal, penetration, accumulated impulses
  Joint.hpp             distance constraint
include/Scene.hpp       bodies, joints, broad phase, solver
include/StepClock.hpp   variable frame time to fixed steps, no SDL dependency
include/Viewport.hpp    world metres to screen pixels, no SDL dependency
include/Renderer.hpp    SDL2 drawing
src/main.cpp            sandbox: event loop and ImGui panel
bench/bench.cpp         headless benchmark
tests/                  Catch2, 33 cases and 1569 assertions
```

`Scene` has no SDL dependency. That is what lets the benchmark run headless,
makes the solver directly testable, and would reduce a WebAssembly build to a
build-configuration change. `StepClock` and `Viewport` are separated for the
same reason and are unit tested.

## Simulation step

`Scene::step(dt)` runs:

1. Integrate every awake body. Symplectic Euler: `a = F/m`, `v += a·dt`, `x += v·dt`, and the equivalent for orientation.
2. Sweep fast bodies against static geometry, stopping each at its first time of impact.
3. Clamp circles to the scene bounds.
4. Broad phase, producing candidate pairs.
5. Narrow phase, producing manifolds.
6. Prepare contacts: fix the restitution target, warm start from the previous step's impulse.
7. Eight solver iterations over contacts and joints.
8. Positional correction.
9. Cache impulses for the next step.
10. Update sleep islands.

Pass 3 predates the walls being real bodies. It is retained because it is the
only thing containing a scene built without walls, so it is a bounds policy
rather than a tunnelling backstop.

## API notes

`Scene::add_circle(position, velocity, acceleration, radius, forces, mass, restitution)`
takes radius before forces.

Restitution is mixed as the product of the two bodies'. A scene where every
restitution is 1.0 never dissipates energy, so nothing settles and a body
resting on an oscillating one can gain speed without bound.

`Box::get_min()` and `get_max()` return the world-space bounding box, which
grows as the box rotates. They match the box itself only at orientation zero;
use `corners()` for the shape and `get_half_body()` for the local extent. The
constructor throws `std::invalid_argument` unless `min < max` on both axes,
since an inverted box yields a negative half extent.

Infinite mass marks a body static: zero inverse mass, so it absorbs no impulse
and is never repositioned.

`Scene::wake` is required after moving a body by hand. A sleeping body skips
integration, so dragging a support out from under a settled stack would leave it
suspended. `wake` rouses the body, its island, and bodies it was touching.
`remove_object` calls it; direct writes to `position` do not.

## Limitations

- Continuous detection covers circles against static geometry only. Two fast
  dynamic bodies can pass through each other, and a fast box is not swept.
- The box sweep treats the grown box's corners as square rather than rounded,
  stopping a body marginally early near a corner.
- No rolling resistance, so a disc that reaches rolling continues indefinitely
  on level ground.
- Box-box warm starting keys on contact-point clipping order, which is not
  stable between frames, so a contact occasionally restarts cold.
- A box balanced on a corner on a slope tumbles rather than settling. This is an
  unstable equilibrium, not a solver fault.
- Static bodies are integrated every step despite never moving.
- Joints are distance constraints only: no hinges, motors or springs.
- No serialisation, and no undo in the sandbox.

## License

No license file, so the project is all rights reserved by default.
