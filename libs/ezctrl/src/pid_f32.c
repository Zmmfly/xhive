/*
 * float (f32) instantiation of the shared PID implementation.
 *
 * The typed entry points are declared by ezctrl/pid.h; this file only selects
 * the scalar type and the name suffix of the shared implementation body.
 */

#include "ezctrl/pid.h"

#define EZC_PID_TYPE float
#define EZC_PID_SUFFIX f32
#include "pid_impl.h"
