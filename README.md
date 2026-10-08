# phys_engine

[![CI](https://github.com/rjbonhagen/phys_engine/actions/workflows/ci.yml/badge.svg)](https://github.com/rjbonhagen/phys_engine/actions/workflows/ci.yml)

A 2D rigid-body physics engine in C++23, plus an SDL2 and Dear ImGui sandbox for
poking at it. Circles, oriented boxes and infinite planes, with friction,
rotation, joints and a spatial-hash broad phase.

I wrote this to understand how physics engines actually work, so the code
favours being readable and checkable over being fast. Where a shortcut would
have been easier, there is usually a comment explaining why it was not taken.

## What it does

The simulation runs on a fixed 1/120 second timestep. Bodies are circles, boxes
or static half-space planes. Contacts are resolved by a sequential impulse
solver with warm starting, Coulomb friction and island sleeping. Fast bodies are
swept against static geometry so they do not pass through thin walls.

Most of the behaviour is checked against closed-form physics rather than against
whatever the code happened to produce. A few examples, all from the test suite:

| what | measured | theory |
| --- | --- | --- |
| Disc rolling down a 30 degree ramp | 3.267 m/s² | `g·sin30 / 1.5` = 3.267 |
| Box sliding to a stop, mu 0.3 | 24.54 m | `v²/(2·mu·g)` = 24.49 m |
| Pendulum period, released horizontal | 3.352 s | 3.350 s for a 90 degree swing |
| Five-link hanging chain, link spacing | 1.0002, 1.0001, 1.0001, 1.0000 | 1.0 |

The disc one is my favourite because it falls out of the solver rather than
being special-cased. Friction at the contact point applies torque, a third of
the work goes into spin because a solid disc has `I = m r² / 2`, and the
acceleration drops to two thirds of the sliding value. The measured contact slip
is exactly zero, which is the definition of rolling without slipping.

## Performance

The broad phase is a uniform spatial hash. All-pairs is kept alongside it as a
reference implementation and is still selectable through
`Scene::set_broad_phase`.

Measured with the headless `bench` target in Release: 120 steps per case,
medians, 0.25-unit circles under gravity in a walled arena, body density held
constant as the count grows.

| bodies | all-pairs | pairs tested | spatial hash | pairs tested | speedup |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 100 | 0.081 ms | 5,356 | 0.044 ms | 138 | 1.9x |
| 500 | 1.873 ms | 126,756 | 0.257 ms | 424 | 7.3x |
| 1,000 | 7.326 ms | 503,506 | 0.485 ms | 693 | 15.1x |
| 2,000 | 30.297 ms | 2,007,006 | 0.885 ms | 1,236 | 34.2x |
| 5,000 | 189.119 ms | 12,517,506 | 2.185 ms | 2,739 | 86.6x |
| 10,000 | not measured | | 4.736 ms | 5,270 | |
| 20,000 | not measured | | 10.694 ms | 10,100 | |

All-pairs is not measured past 5,000 bodies because at 20,000 it is 200 million
pair tests per step and the run takes longer than everything else combined.

At the 16.67 ms budget for 60 Hz, all-pairs runs out at roughly 1,500 bodies,
interpolating between the 1,000 and 2,000 rows. The hash still has headroom at
20,000, where the physics step alone could sustain 94 Hz.

Two caveats. These are numbers from one machine, not a claim about hardware in
general. And holding density constant means this measures the broad phase, not
contact resolution: a dense pile would move the cost into the solver, where the
grid cannot help.

A test runs the same 600-step scene through both broad phases and asserts the
contact count matches on every step, so the grid cannot quietly drop a pair.
That mattered more to me than the speedup. Writing a spatial hash is easy;
proving it did not change behaviour is the part worth having.

```
cmake --build build/windows-x64-debug --config Release --target bench
./build/windows-x64-debug/Release/bench.exe 120
```

## Building

Dependencies (SDL2, Dear ImGui, Catch2) are fetched by CMake at configure time.
Nothing to install.

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

CI builds and runs the suite on Windows, macOS and Linux. One warning about
local builds: use `--clean-first` before trusting a zero-warning claim. An
incremental build will happily skip a translation unit and hide 34 warnings in
it, which is exactly what happened to me for most of a day.

## The sandbox

Modes live in the Controls panel, top left.

Select / Move picks a body and drags it, and walls and ramps are draggable too.
Add Circle and Add Box spawn on click. Add Ramp drags out a slope with a live
preview of the angle. Add Joint takes two clicks: two bodies to connect them, or
a body and then empty space to pin it to the world. Delete Selected does what it
says, as does the Delete key.

Space pauses, period advances one step. The pause and single-step controls exist
because I kept finding bugs by writing throwaway harnesses that stepped at a
fixed rate, and it was obviously silly not to have that in the app.

Show contacts draws contact points and normals, with normal length scaled by
penetration depth. The Width and Height sliders change the world size and the
walls follow.

The window is resizable. The world fits the window rather than the other way
round, so resizing changes the view and never the simulation.

## Layout

```
include/phys/        header-only physics core, no SDL anywhere
  math/Real.hpp      phys::real typedef, currently float
  math/Vec2.hpp      vectors, dot, 2D cross, perpendicular
  Object.hpp         position, velocity, mass, restitution, friction, angular state
  Circle.hpp         circle, with solid-disc inertia
  Box.hpp            oriented box, with rectangle inertia
  Plane.hpp          static half-space, for ramps and ground
  Manifold.hpp       one contact point: normal, penetration, accumulated impulses
  Joint.hpp          distance constraint
include/Scene.hpp    the world: bodies, joints, broad phase, solver
include/StepClock.hpp   variable frame time to fixed steps, no SDL
include/Viewport.hpp    world metres to screen pixels, no SDL
include/Renderer.hpp    SDL2 drawing
src/main.cpp         sandbox: event loop and ImGui panel
bench/bench.cpp      headless benchmark
tests/               Catch2, 33 cases and 1569 assertions
```

`Scene` has no SDL dependency, which is the single decision this project got
most value out of. It is why the benchmark can run headless, why the solver is
testable at all, and why a WebAssembly build would be a build-config change
rather than a port.

`StepClock` and `Viewport` were pulled out for the same reason. `Viewport` in
particular was extracted after I noticed `main.cpp` had its own copy of the
screen-to-world transform. That was fine while the scale was a compile-time
constant and would have broken silently the moment it stopped being one.

## How a step works

`Scene::step(dt)` runs these in order:

1. Integrate every awake body (symplectic Euler: `a = F/m`, `v += a·dt`, `x += v·dt`, and the same for orientation)
2. Sweep fast bodies against static geometry, stopping each at its first time of impact
3. Clamp circles to the scene bounds
4. Broad phase, producing candidate pairs
5. Narrow phase, producing manifolds
6. Prepare contacts: fix the restitution target, warm start from last step's impulse
7. Eight solver iterations over contacts and joints
8. Positional correction
9. Cache impulses for next step's warm start
10. Update sleep islands

Step 3 is the odd one out. It predates the walls being real bodies and is still
the only thing containing a scene built without them, so it stays as a bounds
policy rather than a tunnelling backstop.

## Things that are easy to get wrong

`Scene::add_circle(position, velocity, acceleration, radius, forces, mass, restitution)`
takes radius before forces. I have tripped over this more than once.

Restitution is mixed as the product of the two bodies'. A scene where everything
is 1.0 never dissipates energy and nothing ever settles. This caused the worst
bug in the project's history: a box bouncing forever on an elastic floor
repeatedly slingshot the circle resting on it, like a ball on a swinging racket,
until it reached 586 units per second and left the world. The impulse arithmetic
was correct the whole time.

`Box::get_min()` and `get_max()` return the world-space bounding box, which grows
as the box rotates. They match the box itself only at orientation zero. Use
`corners()` for the real shape and `get_half_body()` for the local extent. The
constructor throws `std::invalid_argument` unless `min < max` on both axes, since
an inverted box produces a negative half extent that quietly breaks every
overlap test.

Infinite mass marks a body as static. It contributes zero inverse mass, so it
absorbs no impulse and is never repositioned.

Moving a body by hand needs `Scene::wake`. A sleeping body skips integration, so
dragging a support out from under a settled stack leaves it hanging in mid air.
`wake` rouses the body, its island, and whatever it was touching.
`remove_object` calls it for you; if you set `position` directly, you have to.

## What it does not do

Being specific about this because the limits are more useful than the features.

Continuous detection covers circles against static geometry only, so two fast
dynamic bodies can still pass through each other and a fast box is not swept at
all.

The box sweep treats the grown box's corners as square rather than rounded, so
it stops a body marginally early near a corner. That errs toward stopping things
rather than letting them through, which is the right direction to be wrong in.

No rolling resistance, so a disc that reaches rolling keeps rolling forever on
level ground.

Box-box warm starting keys on the clipping order of contact points, which is not
perfectly stable between frames, so a contact occasionally restarts cold.

A box balanced on a corner on a slope tumbles rather than settling. That one is
a genuine unstable equilibrium, not a bug, and it fooled me for a while.

Static bodies are still integrated every step even though they never move.
Joints are distance constraints only: no hinges, motors or springs. There is no
serialisation, and the sandbox has no undo.

## License

No license file yet, which means all rights reserved by default. If you want to
use any of this for something, open an issue and I will sort that out.
