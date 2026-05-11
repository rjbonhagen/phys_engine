# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Test

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
- `game/main.cpp` — thin entry point, uses C++23 `std::println`.

Catch2 is fetched via CMake `FetchContent` at configure time (tag v3.5.0) — no manual install needed.
