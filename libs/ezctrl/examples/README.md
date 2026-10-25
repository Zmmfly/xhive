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

From the repository root (POSIX shell). The libraries are object libraries,
so the ezctrl scalar implementation is compiled next to the example together
with the fieldctrl PID it delegates to and fieldctrl's numeric dependencies,
filters and linears:

```sh
# C
cc -std=c99 -Wall -Wextra -I libs/ezctrl/inc -I libs/fieldctrl/include \
   -I libs/filters/include -I libs/linears/include \
   libs/ezctrl/examples/pid_c.c libs/ezctrl/src/pid_f32.c \
   libs/fieldctrl/src/*.c libs/filters/src/*.c libs/linears/src/*.c \
   -lm -o /tmp/pid_c
/tmp/pid_c

# C++ (the C sources are compiled as C into a scratch directory, then linked)
root=$PWD
mkdir -p /tmp/ezctrl_obj
(cd /tmp/ezctrl_obj && cc -std=c99 -c \
    -I "$root/libs/ezctrl/inc" -I "$root/libs/fieldctrl/include" \
    -I "$root/libs/filters/include" -I "$root/libs/linears/include" \
    "$root/libs/ezctrl/src/pid_f32.c" "$root"/libs/fieldctrl/src/*.c \
    "$root"/libs/filters/src/*.c "$root"/libs/linears/src/*.c)
c++ -std=c++11 -Wall -Wextra -I libs/ezctrl/inc -I libs/fieldctrl/include \
    -I libs/linears/include libs/ezctrl/examples/pid_cpp.cpp /tmp/ezctrl_obj/*.o \
    -lm -o /tmp/pid_cpp
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
