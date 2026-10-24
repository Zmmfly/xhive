/*
 * float (f32) instantiation of the shared PID adapter.
 *
 * The typed entry points are declared by ezctrl/pid.h; this file only selects
 * the scalar type, the name suffix shared with fieldctrl_f32_pid_*, and the
 * largest finite value that stands for a disabled output limit.
 */

#include "ezctrl/pid.h"

#include <float.h>

#define EZC_PID_TYPE float
#define EZC_PID_SUFFIX f32
#define EZC_PID_MAX FLT_MAX
#include "pid_impl.h"
