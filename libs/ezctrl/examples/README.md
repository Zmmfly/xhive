# ezctrl examples

Hand-run examples for the library. **They are deliberately not part of any
Xmake target**: nothing here is compiled by `xmake`, `xmake b` or
`xmake test`, so the examples can stay small and readable without adding
build or test time.

| File | Content |
| --- | --- |
| `pid_c.c` | Minimal C PID: zero-initialized context, `ezc_pid_f32_set_gains()`, one update per control period |
| `pid_cpp.cpp` | The same loop through the C++ wrapper: default-constructed `ezc::pid<float>`, `set_params()`, `update()` |

Both close the loop on the same simulated first-order plant and print the
measurement and the controller output, so the two languages can be compared
directly.

## Build and run

From the repository root (POSIX shell). The library is an object library, so
the scalar implementation is compiled right next to the example:

```sh
# C
cc -std=c99 -Wall -Wextra -I libs/ezctrl/inc \
   libs/ezctrl/examples/pid_c.c libs/ezctrl/src/pid_f32.c -o /tmp/pid_c
/tmp/pid_c

# C++ (the C implementation is compiled as C, then linked)
cc -std=c99 -I libs/ezctrl/inc -c libs/ezctrl/src/pid_f32.c -o /tmp/pid_f32.o
c++ -std=c++11 -Wall -Wextra -I libs/ezctrl/inc \
    libs/ezctrl/examples/pid_cpp.cpp /tmp/pid_f32.o -o /tmp/pid_cpp
/tmp/pid_cpp
```

Expected shape of the output (measurement converges to the setpoint of 1.0):

```text
 t [s]    measure     output
  0.00     0.0000     2.0400
  0.50     0.8610     0.9025
  ...        ...        ...
  5.00     0.9998     0.9999
```

## Where the real coverage lives

These examples are documentation, not tests. The behavior they show is
asserted by the host test suites next to the library:

- `libs/ezctrl/test/pid_update.cc` - step semantics, limits, anti-windup, callback
- `libs/ezctrl/test/pid_options.cc` - the optional features
- `libs/ezctrl/test/pid_cpp.cc` - the C++ wrapper
- `libs/ezctrl/test/compile/consumer.c`, `headers.cpp` - C and C++ consumption
