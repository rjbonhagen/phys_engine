#include "Renderer.hpp"
#include <cmath>

Renderer::Renderer(int width, int height, phys::real ppm) : WINDOW_WIDTH(width), WINDOW_HEIGHT(height), PPM(ppm)
{
    if ( SDL_Init(SDL_INIT_EVERYTHING) < 0 )
    {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "SDL init error: %s", SDL_GetError());
        return;
    }

    if ( SDL_CreateWindowAndRenderer(WINDOW_WIDTH, WINDOW_HEIGHT, 0, &WINDOW, &RENDERER) < 0)
    {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Window and renderer creation error: %s", SDL_GetError());
        return;
    }
    SDL_RenderSetVSync(RENDERER, 1);
    valid = true;
}

Renderer::~Renderer()
{
    if (RENDERER) SDL_DestroyRenderer( RENDERER );
    if (WINDOW)   SDL_DestroyWindow( WINDOW );
    SDL_Quit();
}

void Renderer::render_circle(phys::Vec2 p, phys::real r, bool highlight)
{
    p = position_to_screen(p);
    r *= PPM;

    if (highlight)
        SDL_SetRenderDrawColor(RENDERER, 255, 220, 0, 255);
    else
        SDL_SetRenderDrawColor(RENDERER, 255, 255, 255, 255);

    // Midpoint circle, so the span bounds are integral from here on.
    const int cx = static_cast<int>(p.x);
    const int cy = static_cast<int>(p.y);
    const int radius = static_cast<int>(r);

    int x = 0, y = radius, d = 1 - radius;
    while (x <= y)
    {
        int success = 0;
        success |= SDL_RenderDrawLine(RENDERER, cx - x, cy + y, cx + x, cy + y);
        success |= SDL_RenderDrawLine(RENDERER, cx - x, cy - y, cx + x, cy - y);
        success |= SDL_RenderDrawLine(RENDERER, cx - y, cy + x, cx + y, cy + x);
        success |= SDL_RenderDrawLine(RENDERER, cx - y, cy - x, cx + y, cy - x);

        if (success < 0) { SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "render_circle error: %s", SDL_GetError()); }

        if (d < 0) d += 2 * x + 3;
        else       d += 2 * (x - y--) + 5;
        x++;
    }
}

void Renderer::render_arrow(phys::Vec2 from, phys::Vec2 to, SDL_Color color)
{
    phys::Vec2 a = position_to_screen(from);
    phys::Vec2 b = position_to_screen(to);
    SDL_SetRenderDrawColor(RENDERER, color.r, color.g, color.b, color.a);
    SDL_RenderDrawLine(RENDERER, (int)a.x, (int)a.y, (int)b.x, (int)b.y);
}

// A plane is an infinite half-space, so the surface is drawn as a chord long
// enough to leave the viewport, with short ticks on the solid side.
void Renderer::render_plane(phys::Vec2 point, phys::Vec2 normal, bool highlight)
{
    const phys::real length = normal.length();
    if (length == 0.0f) return;

    const phys::Vec2 n{normal.x / length, normal.y / length};
    const phys::Vec2 along{-n.y, n.x};

    // Window diagonal in world units, so the chord always spans the view.
    const phys::real reach = (static_cast<phys::real>(WINDOW_WIDTH) +
                              static_cast<phys::real>(WINDOW_HEIGHT)) / PPM;

    const phys::Vec2 a = position_to_screen(point + along * reach);
    const phys::Vec2 b = position_to_screen(point - along * reach);

    if (highlight) SDL_SetRenderDrawColor(RENDERER, 255, 220, 0, 255);
    else           SDL_SetRenderDrawColor(RENDERER, 180, 180, 255, 255);

    int success = 0;
    success |= SDL_RenderDrawLine(RENDERER, (int)a.x, (int)a.y, (int)b.x, (int)b.y);

    // Hatching: short strokes pointing into the solid side, which is -n.
    const phys::real spacing = 0.5f;
    const phys::real tick    = 0.3f;
    for (phys::real t = -reach; t <= reach; t += spacing)
    {
        const phys::Vec2 root = point + along * t;
        const phys::Vec2 p0   = position_to_screen(root);
        const phys::Vec2 p1   = position_to_screen(root - n * tick);
        success |= SDL_RenderDrawLine(RENDERER, (int)p0.x, (int)p0.y, (int)p1.x, (int)p1.y);
    }

    if (success < 0) { SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "render_plane error: %s", SDL_GetError()); }
}

// Contact markers: a magenta cross at the point, and a cyan normal whose length
// is scaled by penetration so a deep overlap is visible at a glance.
void Renderer::render_contacts(const std::vector<phys::ContactPoint>& contacts)
{
    for (const auto& c : contacts)
    {
        const phys::Vec2 p = position_to_screen(c.point);
        const int x = (int)p.x, y = (int)p.y;
        const int arm = 3;

        SDL_SetRenderDrawColor(RENDERER, 255, 0, 255, 255);
        SDL_RenderDrawLine(RENDERER, x - arm, y - arm, x + arm, y + arm);
        SDL_RenderDrawLine(RENDERER, x - arm, y + arm, x + arm, y - arm);

        // Floor the length so a zero-penetration contact still shows a normal.
        const phys::real length = 0.25f + c.penetration * 4.0f;
        const phys::Vec2 tip = position_to_screen(c.point + c.normal * length);

        SDL_SetRenderDrawColor(RENDERER, 0, 230, 230, 255);
        SDL_RenderDrawLine(RENDERER, x, y, (int)tip.x, (int)tip.y);
    }
}

void Renderer::step(phys::real dt, const std::vector<std::unique_ptr<phys::Object>>& objects, int selected_idx)
{
    update_title(dt);
    render_objects(objects, selected_idx);
}

void Renderer::present()
{
    SDL_RenderPresent(RENDERER);
    SDL_SetRenderDrawColor(RENDERER, 0, 0, 0, 255);
    SDL_RenderClear(RENDERER);
}

void Renderer::render_objects(const std::vector<std::unique_ptr<phys::Object>>& objects, int selected_idx)
{
    for (int i = 0; i < (int)objects.size(); i++)
    {
        bool highlight = (i == selected_idx);

        SDL_Color green = {0, 255, 0, 255};

        if (auto* circle = dynamic_cast<phys::Circle*>(objects[i].get()))
        {
            render_circle(circle->position, circle->radius, highlight);
            render_arrow(circle->position, circle->position + circle->velocity, green);
        }

        if (auto* rect = dynamic_cast<phys::AABB*>(objects[i].get()))
        {
            render_rectangle(rect->get_min(), rect->get_max(), highlight);
            render_arrow(rect->position, rect->position + rect->velocity, green);
        }

        if (auto* plane = dynamic_cast<phys::Plane*>(objects[i].get()))
        {
            render_plane(plane->position, plane->normal, highlight);
        }
    }
}

void Renderer::update_title(phys::real dt)
{
    static float sim_time = 0.0f;
    static Uint64 real_start = SDL_GetPerformanceCounter();
    sim_time += dt;
    float real_time = (SDL_GetPerformanceCounter() - real_start) / (float)SDL_GetPerformanceFrequency();
    float sim_speed = (real_time > 0.0f) ? sim_time / real_time : 1.0f;

    static float smoothed_fps = 0.0f;
    smoothed_fps += (1.0f / dt - smoothed_fps) * 0.1f;

    char title[64];
    SDL_snprintf(title, sizeof(title), "phys_engine | %.0f fps | %.2f ms | sim %.2fx", smoothed_fps, dt * 1000.0f, sim_speed);
    SDL_SetWindowTitle(WINDOW, title);
}

void Renderer::render_rectangle(phys::Vec2 min, phys::Vec2 max, bool highlight)
{
    min = position_to_screen(min);
    max = position_to_screen(max);

    if (highlight)
        SDL_SetRenderDrawColor(RENDERER, 255, 220, 0, 255);
    else
        SDL_SetRenderDrawColor(RENDERER, 255, 255, 255, 255);

    const int x0 = static_cast<int>(min.x), y0 = static_cast<int>(min.y);
    const int x1 = static_cast<int>(max.x), y1 = static_cast<int>(max.y);

    int success = 0;
    success |= SDL_RenderDrawLine(RENDERER, x0, y0, x0, y1);
    success |= SDL_RenderDrawLine(RENDERER, x0, y1, x1, y1);
    success |= SDL_RenderDrawLine(RENDERER, x1, y0, x1, y1);
    success |= SDL_RenderDrawLine(RENDERER, x0, y0, x1, y0);

    if (success < 0) { SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "render_rectangle error: %s", SDL_GetError()); }
}

phys::Vec2 Renderer::position_to_screen(phys::Vec2 p)
{
    return {p.x * PPM, static_cast<phys::real>(WINDOW_HEIGHT) - p.y * PPM};
}
