#pragma once

// GLFW + mjr_render viewer for the MuJoCo backend.
//
// One viewer instance per process. Owns its own thread (GLFW windows must be
// polled from the thread that created them). Reads qpos/qvel from World under
// the world mutex, then renders without holding the lock.
//
// Two implementations: viewer_glfw.cpp (when GLFW is found at configure time)
// and viewer_stub.cpp (when not — start() returns false, stop() is a no-op).

#include <atomic>
#include <thread>

namespace qmini {
namespace hal {
namespace mj {

class Viewer {
public:
    static Viewer& instance();

    // Starts the viewer thread. World must be loaded first (i.e., motor
    // backend started). Returns false if GLFW is unavailable or the window
    // can't be created. Idempotent — second call is a no-op.
    bool start();

    // Joins the viewer thread. Idempotent.
    void stop();

    // Returns true if the user closed the window (so qmini_app can fold
    // and exit the process).
    bool should_close() const { return should_close_.load(); }

    Viewer(const Viewer&) = delete;
    Viewer& operator=(const Viewer&) = delete;

private:
    Viewer() = default;
    ~Viewer();

    void thread_main();

    std::thread th_;
    std::atomic<bool> running_{false};
    std::atomic<bool> should_close_{false};
};

}  // namespace mj
}  // namespace hal
}  // namespace qmini
