# Codebase Review — phys_engine

## Context
Review of the project at `C:\Users\rjbon\repos\phys_engine` as it stands. C++23 / CMake 2D physics engine using SDL2 for rendering and Catch2 for tests.

## What the project is
- Header-only physics core under `include/phys/` (math, Object/Circle/Particle hierarchy, Manifold stub).
- Implementation in `src/` (`Scene.cpp` integration + collision, `Renderer.cpp` SDL drawing, `main.cpp` SDL event loop).
- Tests in `tests/` cover Vec2, Object, Circle, Particle constructors/operators.
- Build: CMake ≥3.10, FetchContent pulls SDL2 2.30.11 and Catch2 3.5.0. Cross-platform presets in `CMakePresets.json`.
- `CLAUDE.md` is the only doc; no README.

## Strengths
- Clean separation: phys library has no SDL dependency.
- FetchContent keeps the build self-contained — no vcpkg/system packages required.
- Math layer (Vec2, Real typedef) is well-covered by tests and easy to extend.
- OOP hierarchy (Object → Circle → Particle) leaves room for new shapes.

## Bugs (verify and fix)
1. ~~**`Manifold.hpp:16-18`** — constructor declares 6 params but the initializer list only initializes 3 (`A`, `B`, `colliding`). `normal`, `penetration`, `contact_point` are left uninitialized. Currently dead code, but a landmine.~~ ✅
2. ~~**`Vec2.hpp:25, 41`** — `operator/` and `normalized()` have no zero guard. `Vec2{0,0}.normalized()` yields NaN. No test exercises this.~~ ✅ (assert guards added)
3. **`Renderer.cpp:57`** — velocity arrow is drawn as `r + (o.velocity * -1)`, so it points opposite to actual motion.
4. ~~**`Scene.cpp:97`** — `circle_vs_circle` collision is detected and logged but no response is applied; circles tunnel through each other. `Manifold` is defined but never constructed.~~ ✅
5. ~~**`Scene.cpp:46`** — reference `dynamic_cast<Circle&>` will throw if ever called on a non-Circle; the surrounding pointer-cast guard makes it safe today, but the reference form is fragile.~~ ✅ (pointer form used throughout)
6. ~~**Type drift** — `Scene::integrate` is `float dt` in the header, `phys::real dt` in the .cpp; `Manifold::penetration` is `float` while the rest of the code uses `phys::real`.~~ ✅

## Design concerns
- ~~**Static global scene state** — `Scene::objects` is `inline static`, so all Scene instances share one world. Blocks isolation and multi-world tests.~~ ✅
- ~~**Renderer ↔ Scene coupling** — Renderer reaches into Scene's static objects directly.~~ ✅
- **Hardcoded constants** — gravity, restitution (`1.0f`, perfect bounce), PPM (`50`), window size, particle defaults all baked into `main.cpp`/`Scene.cpp`. No config struct.
- **O(n²) broad phase** — fine for now, will bite once object counts grow. No spatial partitioning.
- **Euler integration** — acceptable for a starter engine; symplectic Euler or Verlet would be a small change with better energy behavior.
- **Particle vs Circle** — `Particle` is just `Circle` with default mass/radius; the name suggests a distinct concept it doesn't have.

## Smaller items
- ~~`src/Scene.cpp:2` — `#include <iostream>` unused.~~ ✅ (still present in `main.cpp`)
- ~~`Renderer.cpp:37` — `int success` reassigned without checking each return.~~ ✅ (fixed with `|=`)
- `Renderer` constructor swallows SDL_Init / CreateWindow failures silently.
- `main.cpp` has no clean shutdown path — `return 0` jumps out of the loop without explicit teardown.
- No tests for Scene physics or collision; rendering is understandably untested.
- No README for non-Claude readers of the repo.

## Suggested follow-ups

High value, low effort:
- Fix Manifold constructor.
- Add zero-length guard to `Vec2::normalized()` and `operator/`.
- Flip the velocity-arrow sign in `Renderer.cpp:57`.
- Drop the unused `<iostream>` include.

Medium effort:
- Implement collision response (build a Manifold, apply impulse using mass + restitution).
- Make `Scene::objects` an instance member; pass Scene by reference where needed.
- Extract a `PhysicsConfig` (gravity, restitution, PPM) and unify `phys::real` everywhere.

Bigger:
- Add Catch2 tests for `Scene::integrate` and `circle_vs_circle`.
- Swap Euler for semi-implicit Euler / Verlet.
- Add a broad-phase (uniform grid or AABB sweep) once shapes diversify.
