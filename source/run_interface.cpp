#include <cstring>
#include <csignal>
#include <iostream>
#include <string>

#include "user/qmini_app.h"

namespace {
qmini::QminiApp* g_app = nullptr;
void handle_sigint(int) {
    if (g_app) g_app->stop();
}
}  // namespace

int main(int argc, char** argv) {
    qmini::QminiApp::Options opts;

    // CLI: --no-onnx (use identity policy), --keyboard (mode via stdin),
    //      --no-log (skip CSV/UDP), --iface <name>, --policy <path>
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--no-onnx") opts.use_real_onnx = false;
        else if (a == "--keyboard") opts.input_from_keyboard = true;
        else if (a == "--no-log") opts.enable_logging = false;
        else if (a == "--no-viewer") opts.enable_viewer = false;
        else if (a == "--iface" && i + 1 < argc) {
            opts.hw.network_interface = argv[++i];
        } else if (a == "--policy" && i + 1 < argc) {
            opts.policy_path = argv[++i];
        } else if (a == "--mjcf" && i + 1 < argc) {
            opts.hw.mjcf_path = argv[++i];
        } else if (a == "--stand-kp-scale" && i + 1 < argc) {
            opts.stand_kp_scale = std::stof(argv[++i]);
        } else if (a == "--stand-kd-scale" && i + 1 < argc) {
            opts.stand_kd_scale = std::stof(argv[++i]);
        } else {
            std::cerr << "Unknown option: " << a << "\n";
            return 2;
        }
    }

    try {
        qmini::QminiApp app(std::move(opts));
        g_app = &app;
        std::signal(SIGINT,  handle_sigint);
        std::signal(SIGTERM, handle_sigint);
        app.run();
    } catch (const std::exception& e) {
        std::cerr << "fatal: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
