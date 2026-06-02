#include "Renderer.hpp"

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

void Renderer::render_circle(phys::Vec2 p, phys::real r)
{
    p = position_to_screen(p);
    r *= PPM;

    int x = 0, y = r, d = 1 - r;
    while (x <= y)
    {
        SDL_SetRenderDrawColor(RENDERER, 255, 255, 255, 255);
        int success = 0;

        success |= SDL_RenderDrawLine(RENDERER, p.x - x, p.y + y, p.x + x, p.y + y);
        success |= SDL_RenderDrawLine(RENDERER, p.x - x, p.y - y, p.x + x, p.y - y);
        success |= SDL_RenderDrawLine(RENDERER, p.x - y, p.y + x, p.x + y, p.y + x);
        success |= SDL_RenderDrawLine(RENDERER, p.x - y, p.y - x, p.x + y, p.y - x);

        if (success < 0) {  SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "render_circle error: %s", SDL_GetError()); }

        if (d < 0) d += 2 * x + 3;
        else       d += 2 * (x - y--) + 5;
        x++;
    }
}

void Renderer::render_velocity(phys::Object& o)
{
    SDL_SetRenderDrawColor(RENDERER, 0, 255, 0, 255);
    phys::Vec2 r = position_to_screen(o.position);

    phys::Vec2 v = r + (o.velocity*-1);

    int success = SDL_RenderDrawLine(RENDERER, r.x, r.y, v.x, v.y);
    if (success < 0) {  SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "render_velocity error: %s", SDL_GetError()); }
}

void Renderer::step(phys::real dt, const std::vector<std::unique_ptr<phys::Object>>& objects)
{
    update_title(dt);

    SDL_SetRenderDrawColor(RENDERER, 255, 255, 255, 255);
    render_objects(objects);
    SDL_RenderPresent( RENDERER );
    SDL_SetRenderDrawColor(RENDERER, 0, 0, 0, 255);
    SDL_RenderClear( RENDERER );
}

void Renderer::render_objects(const std::vector<std::unique_ptr<phys::Object>>& objects)
{
    for (const auto& p : objects)
    {
        if (auto* circle = dynamic_cast<phys::Circle*>(p.get()))
        {            
            render_circle(circle->position, circle->radius);
            render_velocity(*circle);
        }
        
        if (auto* rect = dynamic_cast<phys::AABB*>(p.get()))
        {
            render_rectangle(rect->get_min(), rect->get_max());
            render_velocity(*rect);
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

void Renderer::render_rectangle(phys::Vec2 min, phys::Vec2 max)
{
    min = position_to_screen(min);
    max = position_to_screen(max);
    SDL_SetRenderDrawColor(RENDERER, 255, 255, 255, 255);

    int success = 0;
    success |= SDL_RenderDrawLine(RENDERER, min.x, min.y, min.x, max.y); // left
    success |= SDL_RenderDrawLine(RENDERER, min.x, max.y, max.x, max.y); // top
    success |= SDL_RenderDrawLine(RENDERER, max.x, min.y, max.x, max.y); // right
    success |= SDL_RenderDrawLine(RENDERER, min.x, min.y, max.x, min.y); // bottom

     if (success < 0) {  SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "render_velocity error: %s", SDL_GetError()); }
}

phys::Vec2 Renderer::position_to_screen(phys::Vec2 p)
{
    return {p.x * PPM, static_cast<phys::real>(WINDOW_HEIGHT) - p.y * PPM};
}