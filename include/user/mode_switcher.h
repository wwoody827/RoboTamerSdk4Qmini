#pragma once

#include <algorithm>
#include <cstdio>
#include <poll.h>
#include <unistd.h>

#include "user/hal/types.h"

namespace qmini {

// HAL-side mode switcher: reads button edges out of hal::JoystickFrame and
// applies the same FSM rules as the pre-HAL code (no transition between
// disjoint mode groups; '3'/'4'/'6'-'9' fold into '3' RL walking).
class ModeSwitcher {
public:
    int rl_task_mode = 0;

    char read_from_joystick(const hal::JoystickFrame& js, char mode) {
        char key = '0';
        if (js.button[9]) key = '1';        // START → ready
        else if (js.button[0]) key = '2';   // A     → stand
        else if (js.button[3]) {            // Y     → RL walk
            rl_task_mode = 3; key = '3';
        } else if (js.button[2]) {          // X     → RL stand
            rl_task_mode = 4; key = '4';
        } else if (js.button[8]) {          // SELECT → sin test
            rl_task_mode = 5; key = '5';
        } else if (js.button[4]) {          // L1
            rl_task_mode = 6; key = '6';
        } else if (js.button[5]) {          // R1
            rl_task_mode = 7; key = '7';
        } else if (js.button[6]) {          // L2
            rl_task_mode = 8; key = '8';
        } else if (js.button[7]) {          // R2
            rl_task_mode = 9; key = '9';
        } else if (js.button[1]) {          // B     → quit
            key = 'q';
        }
        if (key == 'q') return key;
        if (key >= '1') {
            // group [1,2,3+] only allows adjacent moves; '3'+/'4-9' fold to '3'
            const char km = std::min(key,  static_cast<char>('3'));
            const char mm = std::min(mode, static_cast<char>('3'));
            const int diff = (km > mm ? km - mm : mm - km);
            if (diff <= 1 || mode >= '3') {
                if (key >= '3' && key != '5') key = '3';
                return key;
            }
        }
        return mode;
    }

    char read_from_keyboard(char mode) {
        // Non-blocking poll on stdin. Only print the prompt on mode change.
        if (mode != last_prompted_mode_) {
            std::printf("Current mode: %c  Press a digit 1-9 or 'q': ", mode);
            std::fflush(stdout);
            last_prompted_mode_ = mode;
        }
        struct pollfd p{0, POLLIN, 0};
        if (::poll(&p, 1, 0) <= 0 || !(p.revents & POLLIN)) return mode;
        char key = '0';
        int c = std::getchar();
        if (c == EOF) return mode;
        key = static_cast<char>(c);
        while (c != '\n' && c != EOF) c = std::getchar();
        if (key == 'q' && mode == '1') return key;
        if (key >= '1' && key <= '9') {
            if (key == '3') rl_task_mode = 3;
            else if (key == '4') rl_task_mode = 4;
            else if (key == '5') rl_task_mode = 5;
            else if (key == '6') rl_task_mode = 6;
            else if (key == '7') rl_task_mode = 7;
            else if (key == '8') rl_task_mode = 8;
            else if (key == '9') rl_task_mode = 9;
            const char km = std::min(key,  static_cast<char>('3'));
            const char mm = std::min(mode, static_cast<char>('3'));
            const int diff = (km > mm ? km - mm : mm - km);
            if (diff <= 1 || mode >= '3') {
                if (key >= '3' && key != '5') key = '3';
                return key;
            }
        }
        return mode;
    }

private:
    char last_prompted_mode_ = '\0';
public:

    static void print_selected_mode(char mode) {
        switch (mode) {
            case '1': std::printf("\033[32mCurrent mode: folding...\n\033[0m"); break;
            case '2': std::printf("\033[32mCurrent mode: standing...\n\033[0m"); break;
            case '3': std::printf("\033[32mCurrent mode: RL walking...\n\033[0m"); break;
            case '4': std::printf("\033[32mCurrent mode: RL stand / step-in-place...\n\033[0m"); break;
            case '5': std::printf("\033[32mCurrent mode: sin test...\n\033[0m"); break;
            case 'q': std::printf("\033[31mE-stop\n\033[0m"); break;
            default: break;
        }
    }
};

}  // namespace qmini
