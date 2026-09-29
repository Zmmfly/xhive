/*
 * double (f64) instantiation of the shared PID implementation.
 *
 * The typed entry points are declared by ezctrl/pid.h; this file only selects
 * the scalar type and the name suffix of the shared implementation body.
 */

#include "ezctrl/pid.h"

#define EZC_PID_TYPE double
#define EZC_PID_SUFFIX f64
#include "pid_impl.h"
