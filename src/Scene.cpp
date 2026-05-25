#include "Scene.hpp"
#include <iostream>
#include <memory>


Scene::Scene(int width, int height) : SCENE_WIDTH(width), SCENE_HEIGHT(height)
{

}

void Scene::add_particle(phys::Vec2 p, phys::Vec2 v, phys::Vec2 a, phys::Vec2 f, phys::real m)
{
    auto particle = std::make_unique<phys::Particle>();
    particle->position = p;
    particle->velocity = v;
    particle->acceleration = a;
    particle->forces = f;
    particle->mass = m;

    objects.push_back(std::move(particle));
}

void Scene::add_circle(phys::Vec2 p, phys::Vec2 v, phys::Vec2 a, phys::real r, phys::Vec2 f, phys::real m)
{
    auto circle = std::make_unique<phys::Circle>();
    circle->position = p;
    circle->velocity = v;
    circle->acceleration = a;
    circle->radius = r;
    circle->forces = f;
    circle->mass = m;

    objects.push_back(std::move(circle));
}

void Scene::integrate(phys::Object& o, phys::real dt)
{
    o.acceleration = o.forces / o.mass;
    o.velocity += o.acceleration * dt;
    o.position += o.velocity * dt;
}

void Scene::resolve_border_collision_circle(phys::Object& o, phys::real restitution)
{

    phys::Circle& c = dynamic_cast<phys::Circle&>(o);

    if ( (c.position.x - c.radius) < 0.0f )
    {
        c.position.x = c.radius;
        if (c.velocity.x < 0.0f)
        {
            c.velocity.x *= -restitution;
        }
    }
    if ( (c.position.x + c.radius) > SCENE_WIDTH )
    {
        c.position.x = SCENE_WIDTH - c.radius;
        if (c.velocity.x > 0.0f)
        {
            c.velocity.x *= -restitution;
        }
    }
    if (c.position.y - c.radius < 0.0f) {
        c.position.y = c.radius;
        if (c.velocity.y < 0.0f)
        {
            c.velocity.y *= -restitution;
        }
    }

    if (c.position.y + c.radius > SCENE_HEIGHT) {
        c.position.y = SCENE_HEIGHT - c.radius;
        if (c.velocity.y > 0.0f)
        {
            c.velocity.y = -restitution;
        }
    }

}

void Scene::step(phys::real dt)
{ 
    for (const auto& o : objects)
    {
        integrate(*o, dt);

        if (auto* c = dynamic_cast<phys::Circle*>(o.get()))
        {
                    resolve_border_collision_circle(*o, 1.0f);
                    for (const auto& other : objects)
                    {
                        if (*o != *other)
                        {
                            if (auto* c2 = dynamic_cast<phys::Circle*>(other.get()))
                            {
                                if (circle_vs_circle(*c, *c2)) SDL_Log("Circle hit");
                            }

                        }
                    }

        }

    }
}

bool Scene::circle_vs_circle(const phys::Circle& a, const phys::Circle& b) const
{
    phys::Vec2 diff = a.position - b.position;
    phys::real d = diff.length();
    return d <= a.radius + b.radius;
}

