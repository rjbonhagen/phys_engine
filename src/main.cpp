#include <print>
#include "phys/math/Vec2.hpp"
#include "phys/Particle.hpp"


int main()
{

    

    return 0;


}


void Integrate(phys::Particle& p, float dt)
{
    p.acceleration = p.forceAccum / p.mass;
    p.velocity += p.acceleration * dt;
    p.position += p.velocity * dt;
}