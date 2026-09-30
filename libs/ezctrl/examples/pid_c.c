/*
 * Minimal, runnable C example of the ezctrl PID controller.
 *
 * It closes the loop on a simulated first-order plant, so the printed output
 * shows the controller settling on a setpoint. The file is intentionally not
 * part of any Xmake target: build it by hand together with the scalar
 * implementation, for example from the repository root:
 *
 *   cc -std=c99 -Wall -Wextra -I libs/ezctrl/inc \
 *      libs/ezctrl/examples/pid_c.c libs/ezctrl/src/pid_f32.c -o pid_c
 *   ./pid_c
 */

#include "ezctrl/pid.h"

#include <stdio.h>

/* Simulated plant: y' = (u - y) / tau, integrated with the forward Euler rule. */
static float plant_step(float y, float u, float dt, float tau)
{
    return y + (dt / tau) * (u - y);
}

int main(void)
{
    const float dt = 0.01f;      /* 100 Hz control loop */
    const float setpoint = 1.0f;
    const float tau = 0.2f;      /* plant time constant in seconds */
    ezc_pid_f32_t pid = {0};     /* zero-initialized: disabled controller */
    float measure = 0.0f;
    int step;

    /* The whole configuration: kp, ki per second, kd per second. Use kd = 0
     * for a PI loop, or a small value such as 0.02f to add damping. */
    ezc_pid_f32_set_gains(&pid, 2.0f, 4.0f, 0.0f);

    printf("%6s %10s %10s\n", "t [s]", "measure", "output");
    for (step = 0; step <= 500; ++step) {
        const float output = ezc_pid_f32_update(&pid, setpoint, measure, dt);

        if ((step % 50) == 0) {
            printf("%6.2f %10.4f %10.4f\n", (float)step * dt, measure, output);
        }
        measure = plant_step(measure, output, dt, tau);
    }

    /* Optional features are one call each, for example:
     *   ezc_pid_f32_set_limit(&pid, -1.0f, 1.0f);
     *   ezc_pid_f32_set_anti_windup(&pid, true);
     *   ezc_pid_f32_set_derivative_source(&pid, EZC_PID_DERIV_ON_MEASUREMENT);
     *   ezc_pid_f32_set_derivative_filter(&pid, 0.01f);
     *   ezc_pid_f32_set_integral_limit(&pid, -1.0f, 1.0f);
     *   ezc_pid_f32_set_integral_separation(&pid, 2.0f);
     *   ezc_pid_f32_set_setpoint_weight(&pid, 0.0f);
     */
    return 0;
}
