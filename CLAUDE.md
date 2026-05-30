# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Test

### Windows (Visual Studio 2026, x64)

Configure (first time or after CMakeLists changes):
```powershell
cmake --preset "windows-x64-debug"
```

Build everything:
```powershell
cmake --build build/windows-x64-debug --config Debug
```

Run tests:
```powershell
cd build/windows-x64-debug && ctest -C Debug
```

Run tests with output:
```powershell
.\build\windows-x64-debug\Debug\tests.exe
```

Run a single named section:
```powershell
.\build\windows-x64-debug\Debug\tests.exe "[section name]"
```

Run the game:
```powershell
.\build\windows-x64-debug\Debug\Game.exe
```

### macOS (Clang 17, arm64)

Configure (first time or after CMakeLists changes):
```bash
cmake --preset "Clang 17.0.0 arm64-apple-darwin25.1.0"
```

Build everything:
```bash
cmake --build "build/Clang 17.0.0 arm64-apple-darwin25.1.0"
```

Run tests:
```bash
cd "build/Clang 17.0.0 arm64-apple-darwin25.1.0" && ctest
```

Run tests with output:
```bash
cd "build/Clang 17.0.0 arm64-apple-darwin25.1.0" && ./tests
```

Run a single named section:
```bash
cd "build/Clang 17.0.0 arm64-apple-darwin25.1.0" && ./tests "[section name]"
```

Run the game:
```bash
"build/Clang 17.0.0 arm64-apple-darwin25.1.0/Game"
```

## Architecture

`phys` is a header-only library under `include/phys/`. `Scene` lives in `src/Scene.cpp` + `include/Scene.hpp` and has no SDL dependency — both `Game` and `tests` compile it.

### phys library (`include/phys/`)
- `math/Real.hpp` — `phys::real` typedef (`float`). All math types use it so precision can be changed in one place.
- `math/Vec2.hpp` — 2D vector: `+`, `-`, scalar `*`/`/`, `+=`/`-=`, `length()`, `normalize()` (in-place), `normalized()` (returns copy), `dot()` (static), comparison operators (`<`, `<=`, `>`, `>=` compare by length). `assert` guards on zero-division.
- `Object.hpp` — base physics object: `position`, `velocity`, `acceleration`, `forces`, `mass`, `restitution`. `operator==` compares all fields except restitution.
- `Circle.hpp` — extends `Object` with `radius`. Constructor: `(position, velocity, acceleration, forces, mass, restitution, radius)` — no defaults, all required.
- `AABB.hpp` — extends `Object` with `min`/`max` corners. Full 7-arg constructor plus a 2-arg `(min, max)` convenience constructor for static objects (defaults: mass=1, restitution=0.5). Throws `invalid_argument` if min==max.
- `Manifold.hpp` — collision data: pointers to `Object* A/B`, `colliding`, `normal`, `penetration`, `contact_point`.

### Scene (`include/Scene.hpp`, `src/Scene.cpp`)
- Owns objects as `vector<unique_ptr<Object>>`.
- `add_circle(position, velocity, acceleration, radius, forces, mass, restitution)` — note radius comes before forces.
- `add_aabb(min, max)` — creates a static AABB.
- `step(dt)` — integrates all objects (symplectic Euler: `a = F/m`, `v += a*dt`, `x += v*dt`), then resolves border collisions, then detects and resolves circle-circle collisions via impulse.
- Collision response uses the impulse formula: `j = -(1+e)*v_rel·n / (1/mA + 1/mB)`, where `e = restitution_A * restitution_B`.

### Game executable
- `src/main.cpp` — SDL2 event loop. Left-click spawns small circle, middle-click spawns large circle, right-click places AABB. Uses `SDL_Log` for output (console subsystem on Windows).
- `src/Renderer.cpp` / `include/Renderer.hpp` — SDL2 drawing: circles rendered as filled rects scaled by PPM.

### Tests
- `tests/test_vec2.cpp`, `test_object.cpp`, `test_circle.cpp` — unit tests for math/data types.
- `tests/test_scene.cpp` — integration tests for `Scene::step`: velocity integration, force→acceleration, all four border wall bounces, circle-circle overlap detection, collision response (including elastic equal-mass velocity swap).

SDL2 and Catch2 are fetched via CMake `FetchContent` at configure time — no manual install needed.

On Windows, `SDL2d.dll` is copied next to `Game.exe` automatically via a post-build command in `CMakeLists.txt`.

## Communication Style
- Before running PowerShell/Bash commands, briefly state what the command will do in one line
- For C++ project work, prefer explaining concepts alongside edits since user is learning
- Don't switch build approaches (vcpkg → FetchContent → Ninja) mid-task without asking; pick one and stick with it