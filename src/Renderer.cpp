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
}

Renderer::~Renderer()
{
    SDL_DestroyRenderer( RENDERER );
    SDL_DestroyWindow( WINDOW );
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

    int x = 0, y = r, d = 1 - r;
    while (x <= y)
    {
        int success = 0;
        success |= SDL_RenderDrawLine(RENDERER, p.x - x, p.y + y, p.x + x, p.y + y);
        success |= SDL_RenderDrawLine(RENDERER, p.x - x, p.y - y, p.x + x, p.y - y);
        success |= SDL_RenderDrawLine(RENDERER, p.x - y, p.y + x, p.x + y, p.y + x);
        success |= SDL_RenderDrawLine(RENDERER, p.x - y, p.y - x, p.x + y, p.y - x);

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

    int success = 0;
    success |= SDL_RenderDrawLine(RENDERER, min.x, min.y, min.x, max.y);
    success |= SDL_RenderDrawLine(RENDERER, min.x, max.y, max.x, max.y);
    success |= SDL_RenderDrawLine(RENDERER, max.x, min.y, max.x, max.y);
    success |= SDL_RenderDrawLine(RENDERER, min.x, min.y, max.x, min.y);

    if (success < 0) { SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "render_rectangle error: %s", SDL_GetError()); }
}

phys::Vec2 Renderer::position_to_screen(phys::Vec2 p)
{
    return {p.x * PPM, static_cast<phys::real>(WINDOW_HEIGHT) - p.y * PPM};
}
