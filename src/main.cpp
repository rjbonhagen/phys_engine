#include "phys/math/Vec2.hpp"
#include "Scene.hpp"
#include "Renderer.hpp"
#include "StepClock.hpp"
#include <SDL.h>
#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_sdlrenderer2.h>
#include <cmath>

const int   PPM          = 50;   // initial pixels per metre; derived after that
const int   WINDOW_WIDTH = 800;
const int   WINDOW_HEIGHT= 480;
const phys::Vec2 GRAVITY = {0.0f, -9.8f};
const phys::Vec2 ZERO    = {0.0f,  0.0f};

// Spawned bodies keep restitution below 1. resolve_collision uses the product
// of the two restitutions, so at 1.0 everywhere nothing ever dissipates energy:
// a box bounces on the floor forever and slingshots any circle resting on it.


enum class Mode { NORMAL, ADD_CIRCLE, ADD_BOX, ADD_PLANE, ADD_JOINT };

// Perpendicular to the drag, flipped so the solid side always faces downward.
// A plane dragged right-to-left would otherwise come out upside down.
static phys::Vec2 plane_normal_from_drag(phys::Vec2 from, phys::Vec2 to)
{
    phys::Vec2 along = to - from;
    if (along.length() == 0.0f) return {0.0f, 1.0f};

    along = along.normalized();
    const phys::Vec2 n{-along.y, along.x};
    return (n.y < 0.0f) ? phys::Vec2{along.y, -along.x} : n;
}

// Returns index of the topmost non-static object containing world_pos, or -1.
static int hit_test(const std::vector<std::unique_ptr<phys::Object>>& objects, phys::Vec2 p)
{
    // Back to front, so the most recently spawned body wins a click. Static
    // bodies are included: walls and ramps are draggable too. They are added
    // first, so a dynamic body on top of one is still picked in preference.
    for (int i = (int)objects.size() - 1; i >= 0; i--)
    {
        if (auto* pl = dynamic_cast<phys::Plane*>(objects[i].get()))
        {
            // A plane is infinite, so picking it means clicking near its
            // surface rather than inside it.
            const phys::real distance =
                phys::Vec2::dot(p - pl->position, pl->normal);
            if (std::fabs(distance) <= 0.35f) return i;
            continue;
        }

        if (auto* c = dynamic_cast<phys::Circle*>(objects[i].get()))
        {
            phys::Vec2 d = c->position - p;
            if (d.length() <= c->radius) return i;
        }
        else if (auto* a = dynamic_cast<phys::Box*>(objects[i].get()))
        {
            // In the box's own frame, where it is axis-aligned. get_min/get_max
            // are the bounding box now, so using those would select a rotated
            // box from empty space near its corners.
            const phys::Vec2 offset = p - a->position;
            const phys::Vec2 local{phys::Vec2::dot(offset, a->axis_x()),
                                   phys::Vec2::dot(offset, a->axis_y())};
            const phys::Vec2 half = a->get_half_body();

            if (std::fabs(local.x) <= half.x && std::fabs(local.y) <= half.y) return i;
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
    scene.create_walls(T);

    Renderer renderer(WINDOW_WIDTH, WINDOW_HEIGHT, world_w, world_h);
    if (!renderer.is_valid())
    {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "renderer failed to initialise, exiting");
        return 1;
    }

    IMGUI_CHECKVERSION();
    if (!ImGui::CreateContext())
    {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "ImGui context creation failed, exiting");
        return 1;
    }
    ImGui::StyleColorsDark();
    if (!ImGui_ImplSDL2_InitForSDLRenderer(renderer.get_window(), renderer.get_renderer()) ||
        !ImGui_ImplSDLRenderer2_Init(renderer.get_renderer()))
    {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "ImGui backend init failed, exiting");
        ImGui::DestroyContext();
        return 1;
    }

    Mode mode       = Mode::NORMAL;
    int  selected   = -1;
    bool dragging   = false;
    phys::Vec2 drag_offset = ZERO;

    StepClock  clock;              // 1/120 s fixed step
    bool       paused    = false;
    bool       step_once = false;

    bool       placing_plane = false;
    phys::Vec2 plane_start   = ZERO;
    bool       show_contacts = false;

    // First body picked for a joint, or -1 while waiting for one.
    int        joint_first = -1;

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

            if (e.type == SDL_WINDOWEVENT && e.window.event == SDL_WINDOWEVENT_SIZE_CHANGED)
                renderer.on_resize(e.window.data1, e.window.data2);

            if (e.type == SDL_KEYDOWN && !io.WantCaptureKeyboard)
            {
                if (e.key.keysym.sym == SDLK_DELETE && selected != -1)
                {
                    scene.remove_object(selected);
                    selected = -1;
                    dragging = false;
                }
                // Space pauses; period advances one step while paused.
                if (e.key.keysym.sym == SDLK_SPACE)  paused = !paused;
                if (e.key.keysym.sym == SDLK_PERIOD) { paused = true; step_once = true; }
            }

            if (!io.WantCaptureMouse)
            {
                if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_LEFT)
                {
                    phys::Vec2 world = renderer.screen_to_world((phys::real)e.button.x, (phys::real)e.button.y);

                    if (mode == Mode::ADD_CIRCLE)
                    {
                        scene.add_circle(world, ZERO, ZERO, 0.2f, GRAVITY * 1.0f, 1.0f, 0.7f);
                    }
                    else if (mode == Mode::ADD_BOX)
                    {
                        scene.add_box(world - phys::Vec2{0.5f, 0.5f},
                                       world + phys::Vec2{0.5f, 0.5f},
                                       ZERO, ZERO, GRAVITY * 1.0f, 1.0f, 0.6f);
                    }
                    else if (mode == Mode::ADD_PLANE)
                    {
                        plane_start   = world;
                        placing_plane = true;
                    }
                    else if (mode == Mode::ADD_JOINT)
                    {
                        const int picked = hit_test(scene.get_objects(), world);
                        if (picked != -1)
                        {
                            if (joint_first == -1)
                            {
                                joint_first = picked;
                            }
                            else if (picked != joint_first)
                            {
                                auto& objs2 = scene.get_objects();
                                scene.add_joint((size_t)joint_first, (size_t)picked,
                                                objs2[joint_first]->position,
                                                objs2[picked]->position);
                                scene.wake((size_t)joint_first);
                                scene.wake((size_t)picked);
                                joint_first = -1;
                            }
                        }
                        else if (joint_first != -1)
                        {
                            // Second click on empty space pins to the world.
                            scene.add_joint((size_t)joint_first,
                                            scene.get_objects()[joint_first]->position, world);
                            scene.wake((size_t)joint_first);
                            joint_first = -1;
                        }
                    }
                    else
                    {
                        selected = hit_test(scene.get_objects(), world);
                        if (selected != -1)
                        {
                            dragging    = true;
                            drag_offset = scene.get_objects()[selected]->position - world;
                            scene.wake(selected);
                        }
                    }
                }

                if (e.type == SDL_MOUSEBUTTONUP && e.button.button == SDL_BUTTON_LEFT)
                {
                    dragging = false;

                    if (placing_plane)
                    {
                        const phys::Vec2 world = renderer.screen_to_world((phys::real)e.button.x, (phys::real)e.button.y);
                        if ((world - plane_start).length() > 0.1f)
                            scene.add_plane(plane_start, plane_normal_from_drag(plane_start, world), 0.2f);
                        placing_plane = false;
                    }
                }

                if (e.type == SDL_MOUSEMOTION && dragging && selected != -1)
                {
                    phys::Vec2 world = renderer.screen_to_world((phys::real)e.motion.x, (phys::real)e.motion.y);
                    scene.get_objects()[selected]->position = world + drag_offset;
                    scene.get_objects()[selected]->velocity = ZERO;
                    scene.wake(selected);
                }
            }
        }

        Uint64 now = SDL_GetPerformanceCounter();
        float  dt  = (now - prev) / (float)SDL_GetPerformanceFrequency();
        prev = now;

        // Physics advances in fixed increments, so behaviour does not depend on
        // the refresh rate and one slow frame cannot become one huge step.
        int steps = 0;
        if (paused)
        {
            clock.reset();              // do not bank time spent paused
            if (step_once) { steps = 1; step_once = false; }
        }
        else
        {
            steps = clock.advance(dt);
        }

        for (int i = 0; i < steps; i++) scene.step(clock.step_size());

        // Keep dragged object pinned to mouse after physics step
        if (dragging && selected != -1)
        {
            int mx, my;
            SDL_GetMouseState(&mx, &my);
            phys::Vec2 world = renderer.screen_to_world((phys::real)mx, (phys::real)my);
            scene.get_objects()[selected]->position = world + drag_offset;
            scene.get_objects()[selected]->velocity = ZERO;
            scene.wake(selected);
        }

        renderer.step(dt, scene.get_objects(), selected);

        renderer.render_joints(scene.get_joints());

        if (show_contacts) renderer.render_contacts(scene.get_contacts());

        if (placing_plane)
        {
            int mx, my;
            SDL_GetMouseState(&mx, &my);
            const phys::Vec2 world = renderer.screen_to_world((phys::real)mx, (phys::real)my);
            renderer.render_plane(plane_start, plane_normal_from_drag(plane_start, world), true);
        }

        // ImGui frame
        ImGui_ImplSDLRenderer2_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        ImGui::SetNextWindowPos({10, 10});
        ImGui::SetNextWindowSize({210, 0}); // auto height
        ImGui::Begin("Controls", nullptr,
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);

        ImGui::Text("Simulation");
        ImGui::Separator();
        if (ImGui::Button(paused ? "Resume" : "Pause", {-1, 0})) paused = !paused;
        if (!paused) ImGui::BeginDisabled();
        if (ImGui::Button("Step once", {-1, 0})) step_once = true;
        if (!paused) ImGui::EndDisabled();
        ImGui::Text("%.0f Hz fixed step", 1.0f / clock.step_size());
        ImGui::Text("steps this frame: %d", steps);
        if (clock.dropped_time() > 0.0f)
            ImGui::Text("dropped: %.2f s", clock.dropped_time());
        ImGui::TextDisabled("space = pause, . = step");
        ImGui::Checkbox("Show contacts", &show_contacts);
        if (show_contacts) ImGui::Text("contacts: %d", (int)scene.get_contacts().size());

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Text("Mode");
        ImGui::Separator();
        if (ImGui::RadioButton("Select / Move", mode == Mode::NORMAL))    mode = Mode::NORMAL;
        if (ImGui::RadioButton("Add Circle",    mode == Mode::ADD_CIRCLE)) mode = Mode::ADD_CIRCLE;
        if (ImGui::RadioButton("Add Box",      mode == Mode::ADD_BOX))  mode = Mode::ADD_BOX;
        if (ImGui::RadioButton("Add Ramp",      mode == Mode::ADD_PLANE)) mode = Mode::ADD_PLANE;
        if (mode == Mode::ADD_PLANE) ImGui::TextDisabled("drag to set the slope");
        if (ImGui::RadioButton("Add Joint",     mode == Mode::ADD_JOINT)) { mode = Mode::ADD_JOINT; joint_first = -1; }
        if (mode == Mode::ADD_JOINT)
            ImGui::TextDisabled(joint_first == -1 ? "click first body"
                                                  : "click second body, or empty space to pin");

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
                renderer.set_world_size(world_w, world_h);
            }
        }

        ImGui::Spacing();
        ImGui::Separator();

        auto& objs = scene.get_objects();
        int dynamic_count = 0;
        for (const auto& o : objs)
            if (!std::isinf(o->mass)) dynamic_count++;
        ImGui::Text("Objects: %d dynamic, %d total", dynamic_count, (int)objs.size());

        ImGui::Spacing();
        for (int i = 0; i < (int)objs.size(); i++)
        {
            const bool is_static_body = std::isinf(objs[i]->mass);
            const char* type = dynamic_cast<phys::Circle*>(objs[i].get()) ? "Circle"
                             : dynamic_cast<phys::Plane*>(objs[i].get())  ? "Ramp"
                             : is_static_body                             ? "Wall"
                                                                          : "Box";
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
