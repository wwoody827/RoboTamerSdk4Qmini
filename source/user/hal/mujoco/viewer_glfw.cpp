// GLFW-backed MuJoCo viewer thread.

#include "viewer.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>
#include <vector>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include "world.h"

namespace qmini {
namespace hal {
namespace mj {

namespace {

// Render at 60 Hz independent of the control rate.
constexpr int kRenderFps = 60;

// Camera + mouse state. Owned by the viewer thread.
struct CamState {
    mjvCamera  cam;
    mjvOption  opt;
    mjvScene   scene;
    mjrContext context;

    bool button_left   = false;
    bool button_right  = false;
    bool button_middle = false;
    double last_x = 0, last_y = 0;
};

CamState* g_cam = nullptr;  // accessed by GLFW callbacks; one per process

void mouse_button(GLFWwindow* w, int button, int action, int /*mods*/) {
    if (!g_cam) return;
    g_cam->button_left   = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_LEFT)   == GLFW_PRESS;
    g_cam->button_right  = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_RIGHT)  == GLFW_PRESS;
    g_cam->button_middle = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
    glfwGetCursorPos(w, &g_cam->last_x, &g_cam->last_y);
    (void)button; (void)action;
}

void cursor_pos(GLFWwindow* w, double xpos, double ypos) {
    if (!g_cam) return;
    if (!g_cam->button_left && !g_cam->button_right && !g_cam->button_middle) {
        g_cam->last_x = xpos; g_cam->last_y = ypos; return;
    }
    double dx = xpos - g_cam->last_x, dy = ypos - g_cam->last_y;
    g_cam->last_x = xpos; g_cam->last_y = ypos;
    int width, height;
    glfwGetWindowSize(w, &width, &height);
    mjtMouse action;
    bool mod_shift = (glfwGetKey(w, GLFW_KEY_LEFT_SHIFT)  == GLFW_PRESS) ||
                     (glfwGetKey(w, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);
    if (g_cam->button_right)       action = mod_shift ? mjMOUSE_MOVE_H  : mjMOUSE_MOVE_V;
    else if (g_cam->button_left)   action = mod_shift ? mjMOUSE_ROTATE_H: mjMOUSE_ROTATE_V;
    else                            action = mjMOUSE_ZOOM;
    // The world's model pointer is read-only here; no lock needed for cam.
    const mjModel* m = World::instance().model();
    if (!m) return;
    mjv_moveCamera(m, action, dx / height, dy / height, &g_cam->scene, &g_cam->cam);
}

void scroll(GLFWwindow* /*w*/, double /*xoff*/, double yoff) {
    if (!g_cam) return;
    const mjModel* m = World::instance().model();
    if (!m) return;
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoff, &g_cam->scene, &g_cam->cam);
}

}  // namespace

Viewer& Viewer::instance() {
    static Viewer v;
    return v;
}

Viewer::~Viewer() { stop(); }

bool Viewer::start() {
    if (running_.exchange(true)) return true;
    th_ = std::thread([this] { thread_main(); });
    return true;
}

void Viewer::stop() {
    if (!running_.exchange(false)) return;
    if (th_.joinable()) th_.join();
}

void Viewer::thread_main() {
    // Wait briefly for the world to load (motor backend's first send loads it).
    for (int i = 0; i < 200 && running_.load() && !World::instance().loaded(); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const mjModel* m = World::instance().model();
    if (!m) {
        std::fprintf(stderr, "[viewer] world never loaded — viewer disabled\n");
        running_.store(false);
        return;
    }

    if (!glfwInit()) {
        std::fprintf(stderr, "[viewer] glfwInit failed — viewer disabled "
                             "(no display? try DISPLAY=:0)\n");
        running_.store(false);
        return;
    }
    GLFWwindow* window = glfwCreateWindow(1024, 768, "Qmini (MuJoCo)", nullptr, nullptr);
    if (!window) {
        std::fprintf(stderr, "[viewer] glfwCreateWindow failed — viewer disabled\n");
        glfwTerminate();
        running_.store(false);
        return;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    CamState cam{};
    mjv_defaultCamera(&cam.cam);
    cam.cam.distance = 2.0;
    cam.cam.elevation = -20.0;
    cam.cam.azimuth = 135.0;
    mjv_defaultOption(&cam.opt);
    mjv_defaultScene(&cam.scene);
    mjv_makeScene(m, &cam.scene, 2000);
    mjr_defaultContext(&cam.context);
    mjr_makeContext(m, &cam.context, mjFONTSCALE_150);

    g_cam = &cam;
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetCursorPosCallback(window, cursor_pos);
    glfwSetScrollCallback(window, scroll);

    // A scratch mjData on the render thread, populated from the live world
    // snapshot each frame. Lets mjv_updateScene run without holding the
    // world mutex through the (slower) GPU upload + render.
    mjData* render_data = mj_makeData(m);

    std::vector<double> qpos, qvel;
    double sim_time = 0.0;

    auto next_frame = std::chrono::steady_clock::now();
    const auto frame_period = std::chrono::microseconds(1'000'000 / kRenderFps);

    while (running_.load() && !glfwWindowShouldClose(window)) {
        if (World::instance().take_render_snapshot(qpos, qvel, sim_time)) {
            std::memcpy(render_data->qpos, qpos.data(), sizeof(double) * m->nq);
            std::memcpy(render_data->qvel, qvel.data(), sizeof(double) * m->nv);
            render_data->time = sim_time;
            mj_kinematics(m, render_data);
            mj_comPos(m, render_data);
        }
        mjv_updateScene(m, render_data, &cam.opt, nullptr, &cam.cam,
                        mjCAT_ALL, &cam.scene);

        mjrRect viewport{0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        mjr_render(viewport, &cam.scene, &cam.context);

        // Overlay: sim time + camera distance.
        char buf[64];
        std::snprintf(buf, sizeof(buf), "t=%6.2fs", sim_time);
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, buf, nullptr, &cam.context);

        glfwSwapBuffers(window);
        glfwPollEvents();
        next_frame += frame_period;
        std::this_thread::sleep_until(next_frame);
    }
    should_close_.store(true);

    mj_deleteData(render_data);
    mjv_freeScene(&cam.scene);
    mjr_freeContext(&cam.context);
    g_cam = nullptr;

    glfwDestroyWindow(window);
    glfwTerminate();
    running_.store(false);
}

}  // namespace mj
}  // namespace hal
}  // namespace qmini
