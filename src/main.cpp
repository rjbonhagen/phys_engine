#include "phys/math/Vec2.hpp"
#include "Scene.hpp"
#include "Renderer.hpp"
#include <SDL.h>
#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_sdlrenderer2.h>
#include <cmath>

const int   PPM          = 50;
const int   WINDOW_WIDTH = 800;
const int   WINDOW_HEIGHT= 480;
const phys::Vec2 GRAVITY = {0.0f, -9.8f};
const phys::Vec2 ZERO    = {0.0f,  0.0f};

phys::Vec2 screen_to_world(float sx, float sy)
{
    return { sx / PPM, (WINDOW_HEIGHT - sy) / PPM };
}

enum class Mode { NORMAL, ADD_CIRCLE, ADD_AABB };

// Returns index of the topmost non-static object containing world_pos, or -1.
static int hit_test(const std::vector<std::unique_ptr<phys::Object>>& objects, phys::Vec2 p)
{
    for (int i = (int)objects.size() - 1; i >= 0; i--)
    {
        if (std::isinf(objects[i]->mass)) continue;
        if (auto* c = dynamic_cast<phys::Circle*>(objects[i].get()))
        {
            phys::Vec2 d = c->position - p;
            if (d.length() <= c->radius) return i;
        }
        else if (auto* a = dynamic_cast<phys::AABB*>(objects[i].get()))
        {
            if (p.x >= a->get_min().x && p.x <= a->get_max().x &&
                p.y >= a->get_min().y && p.y <= a->get_max().y) return i;
        }
    }
    return -1;
}

int main(int argc, char* argv[])
{
    const phys::real W = (phys::real)WINDOW_WIDTH  / PPM;
    const phys::real H = (phys::real)WINDOW_HEIGHT / PPM;
    const phys::real T = 1.0f;

    phys::real world_w = W;
    phys::real world_h = H;

    Scene scene{ world_w, world_h };
    scene.add_aabb({-T,      -T}, {world_w+T,         0}, ZERO, ZERO, ZERO, INFINITY, 1.0f); // bottom
    scene.add_aabb({-T, world_h}, {world_w+T, world_h+T}, ZERO, ZERO, ZERO, INFINITY, 1.0f); // top
    scene.add_aabb({-T,      -T}, {        0, world_h+T}, ZERO, ZERO, ZERO, INFINITY, 1.0f); // left
    scene.add_aabb({world_w, -T}, {world_w+T, world_h+T}, ZERO, ZERO, ZERO, INFINITY, 1.0f); // right

    auto& wall_objs   = scene.get_objects();
    auto* wall_bottom = static_cast<phys::AABB*>(wall_objs[0].get());
    auto* wall_top    = static_cast<phys::AABB*>(wall_objs[1].get());
    auto* wall_left   = static_cast<phys::AABB*>(wall_objs[2].get());
    auto* wall_right  = static_cast<phys::AABB*>(wall_objs[3].get());

    wall_bottom->position += {0.0f, 2.0f};
    wall_left->position += {2.0f, 0.0f};
    wall_top->position -= {0.0f, 2.0f};
    wall_right->position -=  {2.0f, 0.0f};



    Renderer renderer(WINDOW_WIDTH, WINDOW_HEIGHT, PPM);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplSDL2_InitForSDLRenderer(renderer.get_window(), renderer.get_renderer());
    ImGui_ImplSDLRenderer2_Init(renderer.get_renderer());

    Mode mode       = Mode::NORMAL;
    int  selected   = -1;
    bool dragging   = false;
    phys::Vec2 drag_offset = ZERO;

    Uint64 prev = SDL_GetPerformanceCounter();
    bool   quit = false;

    while (!quit)
    {
        SDL_Event e;
        while (SDL_PollEvent(&e))
        {
            ImGui_ImplSDL2_ProcessEvent(&e);
            ImGuiIO& io = ImGui::GetIO();

            if (e.type == SDL_QUIT)
                quit = true;

            if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_DELETE && selected != -1)
            {
                scene.remove_object(selected);
                selected = -1;
                dragging = false;
            }

            if (!io.WantCaptureMouse)
            {
                if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_LEFT)
                {
                    phys::Vec2 world = screen_to_world((float)e.button.x, (float)e.button.y);

                    if (mode == Mode::ADD_CIRCLE)
                    {
                        scene.add_circle(world, ZERO, ZERO, 0.2f, GRAVITY * 0.001f, 0.001f, 1.0f);
                    }
                    else if (mode == Mode::ADD_AABB)
                    {
                        scene.add_aabb(world - phys::Vec2{0.5f, 0.5f},
                                       world + phys::Vec2{0.5f, 0.5f},
                                       ZERO, ZERO, GRAVITY * 1.0f, 1.0f, 1.0f);
                    }
                    else
                    {
                        selected = hit_test(scene.get_objects(), world);
                        if (selected != -1)
                        {
                            dragging    = true;
                            drag_offset = scene.get_objects()[selected]->position - world;
                        }
                    }
                }

                if (e.type == SDL_MOUSEBUTTONUP && e.button.button == SDL_BUTTON_LEFT)
                    dragging = false;

                if (e.type == SDL_MOUSEMOTION && dragging && selected != -1)
                {
                    phys::Vec2 world = screen_to_world((float)e.motion.x, (float)e.motion.y);
                    scene.get_objects()[selected]->position = world + drag_offset;
                    scene.get_objects()[selected]->velocity = ZERO;
                }
            }
        }

        Uint64 now = SDL_GetPerformanceCounter();
        float  dt  = (now - prev) / (float)SDL_GetPerformanceFrequency();
        prev = now;

        scene.step(dt);

        // Keep dragged object pinned to mouse after physics step
        if (dragging && selected != -1)
        {
            int mx, my;
            SDL_GetMouseState(&mx, &my);
            phys::Vec2 world = screen_to_world((float)mx, (float)my);
            scene.get_objects()[selected]->position = world + drag_offset;
            scene.get_objects()[selected]->velocity = ZERO;
        }

        renderer.step(dt, scene.get_objects(), selected);

        // ImGui frame
        ImGui_ImplSDLRenderer2_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        ImGui::SetNextWindowPos({10, 10});
        ImGui::SetNextWindowSize({170, 0}); // auto height
        ImGui::Begin("Controls", nullptr,
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);

        ImGui::Text("Mode");
        ImGui::Separator();
        if (ImGui::RadioButton("Select / Move", mode == Mode::NORMAL))    mode = Mode::NORMAL;
        if (ImGui::RadioButton("Add Circle",    mode == Mode::ADD_CIRCLE)) mode = Mode::ADD_CIRCLE;
        if (ImGui::RadioButton("Add AABB",      mode == Mode::ADD_AABB))  mode = Mode::ADD_AABB;

        ImGui::Spacing();

        bool has_selection = (selected != -1);
        if (!has_selection) ImGui::BeginDisabled();
        if (ImGui::Button("Delete Selected", {-1, 0}))
        {
            scene.remove_object(selected);
            selected = -1;
            dragging = false;
        }
        if (!has_selection) ImGui::EndDisabled();

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Text("World Size");
        {
            float fw = world_w, fh = world_h;
            bool changed = false;
            changed |= ImGui::SliderFloat("Width",  &fw, 4.0f, 32.0f, "%.1f m");
            changed |= ImGui::SliderFloat("Height", &fh, 3.0f, 20.0f, "%.1f m");
            if (changed)
            {
                world_w = fw;
                world_h = fh;
                scene.set_dimensions(world_w, world_h);
                wall_bottom->resize({-T,      -T}, {world_w+T,         0});
                wall_top->resize   ({-T, world_h}, {world_w+T, world_h+T});
                wall_left->resize  ({-T,      -T}, {        0, world_h+T});
                wall_right->resize ({world_w, -T}, {world_w+T, world_h+T});
            }
        }

        ImGui::Spacing();
        ImGui::Separator();

        auto& objs = scene.get_objects();
        int dynamic_count = 0;
        for (const auto& o : objs)
            if (!std::isinf(o->mass)) dynamic_count++;
        ImGui::Text("Objects: %d", dynamic_count);

        ImGui::Spacing();
        for (int i = 0; i < (int)objs.size(); i++)
        {
            if (std::isinf(objs[i]->mass)) continue;
            const char* type = dynamic_cast<phys::Circle*>(objs[i].get()) ? "Circle" : "AABB";
            char label[32];
            SDL_snprintf(label, sizeof(label), "%s %d", type, i);
            if (ImGui::Selectable(label, selected == i))
                selected = i;
        }

        ImGui::End();
        ImGui::Render();
        ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), renderer.get_renderer());

        renderer.present();
    }

    ImGui_ImplSDLRenderer2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

    return 0;
}
