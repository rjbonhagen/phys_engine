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

`phys` is a header-only library under `include/phys/`. The `Game` executable and `tests` executable both link against it.

- `include/phys/math/Real.hpp` — defines `phys::real` as a `float` typedef. All math types use this so precision can be changed in one place.
- `include/phys/math/Vec2.hpp` — 2D vector with `operator+`, `operator-`, scalar `operator*`/`operator/`, `length()`, `normalize()`, and dot product via `operator*(Vec2)`.
- `tests/test_vec2.cpp` — Catch2 v3 tests. Uses `Catch::Approx` for all floating point comparisons involving `sqrt`.
- `src/main.cpp` — SDL2 entry point with event loop. Uses `SDL_Log`/`SDL_LogError` for output (stdout is suppressed in Windows GUI apps).

SDL2 and Catch2 are fetched via CMake `FetchContent` at configure time — no manual install needed.

On Windows, `SDL2d.dll` is copied next to `Game.exe` automatically via a post-build command in `CMakeLists.txt`.