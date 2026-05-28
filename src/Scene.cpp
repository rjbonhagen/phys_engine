#include "Scene.hpp"
#include <iostream>
#include <memory>


Scene::Scene(int width, int height) : SCENE_WIDTH(width), SCENE_HEIGHT(height)
{

}

void Scene::add_circle(phys::Vec2 p, phys::Vec2 v, phys::Vec2 a, phys::real r, phys::Vec2 f, phys::real m, phys::real rest)
{
    auto circle = std::make_unique<phys::Circle>();
    circle->position = p;
    circle->velocity = v;
    circle->acceleration = a;
    circle->radius = r;
    circle->forces = f;
    circle->mass = m;
    circle->restitution = rest;

    objects.push_back(std::move(circle));
}

void Scene::integrate(phys::Object& o, phys::real dt)
{
    o.acceleration = o.forces / o.mass;
    o.velocity += o.acceleration * dt;
    o.position += o.velocity * dt;
}

void Scene::resolve_border_collision_circle(phys::Circle& c)
{
    if ( (c.position.x - c.radius) < 0.0f )
    {
        c.position.x = c.radius;
        if (c.velocity.x < 0.0f)
        {
            c.velocity.x *= -c.restitution;
        }
    }
    if ( (c.position.x + c.radius) > SCENE_WIDTH )
    {
        c.position.x = SCENE_WIDTH - c.radius;
        if (c.velocity.x > 0.0f)
        {
            c.velocity.x *= -c.restitution;
        }
    }
    if (c.position.y - c.radius < 0.0f) {
        c.position.y = c.radius;
        if (c.velocity.y < 0.0f)
        {
            c.velocity.y *= -c.restitution;
        }
    }

    if (c.position.y + c.radius > SCENE_HEIGHT) {
        c.position.y = SCENE_HEIGHT - c.radius;
        if (c.velocity.y > 0.0f)
        {
            c.velocity.y *= -c.restitution;
        }
    }

}

void Scene::step(phys::real dt)
{ 

    for (const auto& o : objects) { integrate(*o, dt); }


    std::vector<phys::Manifold> manifolds;

    for (const auto& o: objects)
    {
        if (auto* c = dynamic_cast<phys::Circle*>(o.get()))
        {
            resolve_border_collision_circle(*c);
            for (const auto& other : objects)
            {
                if (*o != *other)
                {
                    if (auto* c2 = dynamic_cast<phys::Circle*>(other.get()))
                    {
                        if (circle_vs_circle(*c, *c2)) 
                        {
                            phys::Vec2 norm = (c->position - c2->position).normalized();
                            manifolds.push_back(phys::Manifold(c, c2, true, norm, 0.0f, c->position + norm*c->radius));
                        }
                    }

                }
            }

        }
    }

    for (auto& m : manifolds)
    {
        resolve_collision(m);
    }
    
}

void Scene::resolve_collision(phys::Manifold m)
{
    if (!m.colliding) return;
    SDL_Log("Resolved");
    phys::real restitution = m.A->restitution * m.B->restitution;

    phys::Vec2 v_ap = m.A->velocity.dot(m.normal*-1);
    phys::Vec2 v_bp = m.B->velocity.dot(m.normal);

    phys::Vec2 v_ab = v_ap - v_bp;


    phys::real impulse = ((v_ab) * -(1 + restitution)).dot(m.normal) / m.normal.dot(m.normal*(1/m.A->mass + 1/m.B->mass));

    m.A->velocity += m.normal*(impulse / m.A->mass);
    m.B->velocity -= m.normal*(impulse / m.B->mass);

    m.colliding = false;
}

bool Scene::circle_vs_circle(const phys::Circle& a, const phys::Circle& b) const
{
    phys::Vec2 diff = a.position - b.position;
    phys::real d = diff.length();
    return d <= a.radius + b.radius;
}

