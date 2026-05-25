// No-op viewer (linked when GLFW isn't found at configure time).

#include "viewer.h"

#include <cstdio>

namespace qmini {
namespace hal {
namespace mj {

Viewer& Viewer::instance() {
    static Viewer v;
    return v;
}

Viewer::~Viewer() = default;

bool Viewer::start() {
    std::fprintf(stderr, "[viewer] not built (libglfw3-dev was missing at "
                         "configure time). Reconfigure with glfw available.\n");
    return false;
}

void Viewer::stop() {}

}  // namespace mj
}  // namespace hal
}  // namespace qmini
