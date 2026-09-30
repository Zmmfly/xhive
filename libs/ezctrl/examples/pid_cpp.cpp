/*
 * Minimal, runnable C++ example of the ezctrl PID wrapper (ezctrl.hpp).
 *
 * The same simulated plant as the C example, so the two can be compared
 * directly. The file is intentionally not part of any Xmake target: build it
 * by hand from the repository root, for example:
 *
 *   cc -std=c99 -I libs/ezctrl/inc -c libs/ezctrl/src/pid_f32.c -o pid_f32.o
 *   c++ -std=c++11 -Wall -Wextra -I libs/ezctrl/inc \
 *       libs/ezctrl/examples/pid_cpp.cpp pid_f32.o -o pid_cpp
 *   ./pid_cpp
 */

#include "ezctrl.hpp"

#include <cstdio>

namespace {

/* Simulated plant: y' = (u - y) / tau, integrated with the forward Euler rule. */
float plant_step(float y, float u, float dt, float tau) {
    return y + (dt / tau) * (u - y);
}

}  // namespace

int main() {
    const float dt = 0.01f;    // 100 Hz control loop
    const float setpoint = 1.0f;
    const float tau = 0.2f;    // plant time constant in seconds
    ezc::pid<float> ctrl;      // default-constructed: disabled controller
    float measure = 0.0f;

    // The whole configuration: kp, ki per second, kd per second. Use kd = 0
    // for a PI loop, or a small value such as 0.02f to add damping.
    ctrl.set_params(2.0f, 4.0f, 0.0f);

    std::printf("%6s %10s %10s\n", "t [s]", "measure", "output");
    for (int step = 0; step <= 500; ++step) {
        const float output = ctrl.update(setpoint, measure, dt);

        if ((step % 50) == 0) {
            std::printf("%6.2f %10.4f %10.4f\n", static_cast<float>(step) * dt, measure,
                        output);
        }
        measure = plant_step(measure, output, dt, tau);
    }

    // Optional features enable themselves when they are configured, for example:
    //   ctrl.limit(-1.0f, 1.0f);
    //   ctrl.anti_windup(true);
    //   ctrl.derivative_source(EZC_PID_DERIV_ON_MEASUREMENT);
    //   ctrl.derivative_filter(0.01f);
    //   ctrl.integral_limit(-1.0f, 1.0f);
    //   ctrl.integral_separation(2.0f);
    //   ctrl.setpoint_weight(0.0f);
    return 0;
}
