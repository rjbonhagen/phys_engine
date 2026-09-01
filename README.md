# phys_engine

A small 2D physics engine written in C++23, with an SDL2 + Dear ImGui sandbox for
experimenting with circle and AABB collisions in real time.

## Features

- Symplectic Euler integration (`a = F/m`, `v += a·dt`, `x += v·dt`)
- Circle vs. circle and circle/AABB vs. AABB collision detection
- Impulse-based collision response with configurable restitution
- Resizable world bounds with static AABB walls
- Interactive sandbox: spawn circles/AABBs, select, drag, and delete objects via an
  ImGui control panel

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

See [CLAUDE.md](CLAUDE.md) for further architecture notes and build details aimed at
AI coding assistants working in this repo.
