#include <print>
#include "phys/math/Vec2.hpp"

int main()
{
    phys::Vec2 v(1.0f, 2.0f);
    std::println("hello from main");
    std::println("Vector x: {}, y: {}", v.x, v.y);

    return 0;
}
