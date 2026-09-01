#include "Scene.hpp"
#include <memory>
#include <stdexcept>

#include <iostream>

Scene::Scene(phys::real width, phys::real height) : SCENE_WIDTH(width), SCENE_HEIGHT(height)
{

}

void Scene::add_circle(phys::Vec2 p, phys::Vec2 v, phys::Vec2 a, phys::real r, phys::Vec2 f, phys::real m, phys::real rest)
{
    if (m <= 0.0f) throw std::invalid_argument("mass must be positive");
    auto circle = std::make_unique<phys::Circle>(p, v, a, f, m, rest, r);
    objects.push_back(std::move(circle));
}

void Scene::add_aabb(phys::Vec2 min, phys::Vec2 max, phys::Vec2 velocity, phys::Vec2 acceleration, phys::Vec2 forces, phys::real mass, phys::real restitution)
{
    if (mass <= 0.0f) throw std::invalid_argument("mass must be positive");
    auto box = std::make_unique<phys::AABB>(min, max, velocity, acceleration, forces, mass, restitution);
    objects.push_back(std::move(box));
}


void Scene::remove_object(size_t index)
{
    if (index < objects.size())
        objects.erase(objects.begin() + index);
}

void Scene::integrate(phys::Object& o, phys::real dt)
{
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

    for (const auto& o : objects) 
    { 
        o->acceleration = o->forces / o->mass;
        integrate(*o, dt); 
    }


    std::vector<phys::Manifold> manifolds;

    for (const auto& o: objects)
    {
        if (auto* circ = dynamic_cast<phys::Circle*>(o.get()))
        {
            resolve_border_collision_circle(*circ);
            for (const auto& other : objects)
            {
                if (o.get() != other.get())
                {
                    if (auto* circ2 = dynamic_cast<phys::Circle*>(other.get()))
                    {
                        phys::real penetration = 0.0f;
                        if (circle_vs_circle(*circ, *circ2, penetration))
                        {
                            phys::Vec2 diff = circ->position - circ2->position;
                            phys::Vec2 norm = (diff.length() > 0.0f) ? diff.normalized() : phys::Vec2{1, 0};
                            manifolds.push_back(phys::Manifold(circ, circ2, true, norm, penetration, circ->position + norm*circ->radius));
                        }
                    }

                }
            }

        }

        if (auto* rect = dynamic_cast<phys::AABB*>(o.get()))
        {
            for (const auto& other : objects)
            {
                if (o.get() != other.get())
                {
                    if (auto* rect2 = dynamic_cast<phys::AABB*>(other.get()))
                    {
                        phys::Vec2 norm{0, 0};
                        phys::real penetration = 0.0f;
                        if (aabb_vs_aabb(*rect, *rect2, norm, penetration))
                        {
                            manifolds.push_back(phys::Manifold(rect, rect2, true, norm, penetration, {0,0}));
                        }
                    }
                }
            }

        }
    }

    for (auto& m : manifolds)
    {
        if (m.colliding)
        {
            resolve_collision(m);
        }
    }
    
}

void Scene::resolve_collision(phys::Manifold& m)
{
    if (!m.colliding) return;

    phys::Object* A = m.A;
    phys::Object* B = m.B;

    phys::real restitution = A->restitution * B->restitution;

    phys::Vec2 v_ab = A->velocity - B->velocity;

    phys::real vel_normal = phys::Vec2::dot(v_ab, m.normal); 

    if (vel_normal > 0) return;

    phys::real j = -(1.0f + restitution) * vel_normal;
    if (j != 0) j /= 1/A->mass + 1/B->mass;

    phys::Vec2 impulse = m.normal * j;

    A->velocity += impulse / A->mass;
    B->velocity -= impulse / B->mass;

    phys::real total_invmass = 1/A->mass + 1/B->mass;

    if (total_invmass > 0.0f)
    {
        const phys::real percent = 0.8f;
        const phys::real slop = 0.01f;

        phys::real correction_mag = std::max(m.penetration - slop, 0.0f) / total_invmass * percent;
        phys::Vec2 correction = m.normal * correction_mag;

        A->position += correction / A->mass;
        B->position -= correction / B->mass;
    }

    m.colliding = false;
}

bool Scene::circle_vs_circle(const phys::Circle& a, const phys::Circle& b, phys::real& penetration) const
{
    phys::Vec2 diff = a.position - b.position;
    phys::real d = diff.length();
    if (d <= a.radius + b.radius)
    {
        penetration = a.radius + b.radius - d;
        return true;
    }
    else return false;
}

bool Scene::aabb_vs_aabb(const phys::AABB& a, const phys::AABB& b, phys::Vec2& norm, phys::real& penetration) const {
    phys::Vec2 d = b.position - a.position;

    phys::real x_overlap = a.get_half_body().x + b.get_half_body().x - fabs(d.x);
    phys::real y_overlap = a.get_half_body().y + b.get_half_body().y - fabs(d.y);

    if (x_overlap <= 0 || y_overlap <= 0) return false;

    if (x_overlap < y_overlap)
    {
        norm = (d.x < 0) ? phys::Vec2{1, 0} : phys::Vec2{-1, 0};
        penetration = x_overlap;
    }
    else
    {
        norm = (d.y < 0) ? phys::Vec2{0, 1} : phys::Vec2{0, -1};
        penetration = y_overlap;
    }

    return true;
}

bool Scene::aabb_vs_circle(const phys::AABB& a, const phys::Circle& c, phys::real& penetration) const
{
    return false;
}
