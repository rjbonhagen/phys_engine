#include "Scene.hpp"

#include <iostream>

bool Scene::add_object(phys::Object& o)
{
    objects.push_back( &o );
    return true;
}
void Scene::integrate(phys::Object* o, phys::real dt)
{
    o->acceleration = o->forces / o->mass;
    o->velocity += o->acceleration * dt;
    o->position += o->velocity * dt;

    if (o->position.y >= 480 || o->position.y <= 0)
    {
        o->velocity.y *= -1;
    }

    if (o->position.x >= 640 || o->position.x <= 0)
    {
        o->velocity.x *= -1;
    }

    SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "velocity : %f", o->velocity.y);
}

void Scene::step(phys::real dt)
{
    for (phys::Object* o: objects)
    {
        integrate(o, dt);
    }
}