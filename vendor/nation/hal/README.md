# Nation SHRTIM HAL (N32H474)

Unit-centric modern C API over the N32H47x/48x vendor std-periph SHRTIM
driver. One timing unit (A..F) owns one PWM period and one complementary
output pair; a handful of calls cover the usual power-electronics needs.

```c
#include "nation_shrtim.h"

shrtim_init(0);                       /* SHRTPLL -> 240 MHz kernel clock */
shrtim_pinmux(SHRTIM_OUT_C1);         /* fixed AF on the silicon pins     */
shrtim_pinmux(SHRTIM_OUT_C2);

shrtim_pwm_cfg_t cfg = SHRTIM_PWM_CFG_DEFAULT;
cfg.freq_hz    = 65000u;              /* 65 kHz                           */
cfg.dt_rise_ns = 100u;                /* 100 ns symmetric dead-time       */
cfg.dt_fall_ns = 100u;
shrtim_pwm_init(SHRTIM_UNIT_C, &cfg); /* unit left stopped                */
shrtim_pwm_start(SHRTIM_UNIT_C);      /* outputs run (enable_outputs)     */
```

## API surface (42 functions)

| Group | Functions |
| --- | --- |
| Lifecycle | `shrtim_init` / `shrtim_clk_hz` / `shrtim_deinit` |
| Pin muxing | `shrtim_pinmux` / `shrtim_pinmux_fault` |
| PWM config & run | `shrtim_pwm_init` / `_freq` / `_freq_get` / `_duty` / `_duty2` / `_deadtime` / `_start` / `_stop` / `_start_all` / `_stop_all` / `_oneshot_arm` |
| Output control | `shrtim_out_enable` / `_disable` / `_force` / `_polarity`, `shrtim_pwm_swap` / `_pushpull` / `_chopper` |
| Raw access | `shrtim_counter` / `shrtim_period_raw` / `shrtim_compare_raw` / `shrtim_compare_get` |
| Master & triggers | `shrtim_master_period` / `shrtim_pwm_sync_to_master` / `shrtim_adc_trigger` |
| EXEV / fault / capture | `shrtim_exev_config` / `shrtim_exev_reset_unit`, `shrtim_fault_config` / `_attach` / `_reset`, `shrtim_capture_config` / `shrtim_capture_read` |
| Interrupts | `shrtim_irq_hook` / `shrtim_irq_enable` / `shrtim_isr_master` / `shrtim_isr_timer` / `shrtim_isr_fault` |

Configuration functions that can fail return `0` (`SHRTIM_OK`) or a
negative `shrtim_rc_t`; level/run helpers such as `shrtim_out_force()` or
`shrtim_pwm_start()` are `void`. Full contracts in `inc/nation_shrtim.h`
(Doxygen).

## Fixed output pins

| Unit | x1 | x2 | | Unit | x1 | x2 |
| --- | --- | --- | --- | --- | --- | --- |
| A | A1 = PA8 | A2 = PA9 | | D | D1 = PB14 | D2 = PB15 |
| B | B1 = PA10 | B2 = PA11 | | E | E1 = PC8 | E2 = PC9 |
| C | C1 = PB12 | C2 = PB13 | | F | F1 = PC6 | F2 = PC7 |

Note: A2/B1 (PA9/PA10) are also USART1 pads — see the examples for the
pin-sharing caveat.

## Semantics worth knowing (hardware-validated on N32H474RC)

- `shrtim_pwm_init()` leaves the unit stopped; call `shrtim_pwm_start()`.
  Runtime `shrtim_pwm_freq()/duty()` changes are glitch-free via preload
  per unit; changing a multi-unit group updates members one after another
  (transient window), and master-locked CENTER pairs keep a startup-
  dependent cross-unit phase residual (see the examples).
- Center alignment (`SHRTIM_ALIGN_CENTER`) locks the unit to the master
  period anchor; pulses are exact in frequency/duty and x1/x2 split the
  period exactly, but the pulse is anchored at the period edge rather than
  geometrically centered (silicon crossbar limitation, see
  `center_compare()` docs in the source).
- Multi-unit phase alignment: prefer CENTER alignment for phase-locked
  groups — `shrtim_pwm_init()`/`shrtim_pwm_freq()` configure and keep
  following the shared master anchor automatically (hardware-validated
  path; all CENTER units must share one frequency). Manual
  `shrtim_master_period()` anchoring writes raw high-resolution ticks and
  does NOT touch the master prescaler, so hand-computed tick values are
  only correct against a matching master time base — treat it as a raw
  escape hatch, not the default path.
- UPDATE interrupts fire 2-4x the nominal per-period rate (dual-edge preload
  transfer semantics) — plan IRQ budgets accordingly.
- Interrupts are manual-hookup by design: the application defines the
  `SHRT1_MST/TIMx/FLT` vectors and calls `shrtim_isr_master() /
  shrtim_isr_timer(unit) / shrtim_isr_fault()` from them.
- Counter frequency floor: with the slowest high-resolution prescaler the
  minimum PWM frequency is roughly `fSHRTIM / 4 / 65536` (~916 Hz at
  240 MHz). Line-frequency legs (totem-pole PFC slow leg) belong on
  `shrtim_out_force()` or GPIO, not on PWM units — see the bridgeless PFC
  example.

## Examples

`examples/` holds configuration skeletons for common single-chip
topologies (each file maps units/outputs to switches and shows the runtime
control hooks; compile-checked with `arm-none-eabi-gcc -std=c99 -Wall
-Wextra`). The HAL primitives they use are hardware-validated; the
topology mappings themselves are illustrative and still need
power-stage-specific verification (commutation blanking, interleave phase,
sampling points) before driving real hardware:

| File | Topology |
| --- | --- |
| `shrtim_example_half_pfc_full_llc.c` | Half-bridge PFC (2 PWM) + full-bridge LLC (4 PWM, CENTER-anchored, swap-based diagonal) |
| `shrtim_example_bridgeless_pfc.c` | Totem-pole bridgeless PFC (4 outputs, force-commutated slow leg) |
| `shrtim_example_full_llc.c` | Full-bridge LLC (4 PWM, PFM sweep with auto anchor) |
| `shrtim_example_full_pfc_half_llc.c` | Two NON-interleaved PFC fast legs (4 PWM, independent channels) + half-bridge LLC (2 PWM) |

## Build integration

The HAL registers as `nation_hal` (object target) under the
`NATION_USE_N32H47X_48X` kconfig guard and is pulled in through the
`vendor_nation` dependency chain. Reference integration and the hardware
verification matrix live in the demo project
(`xhive_testspace/n32h474_demo`, see its `tools/VERIFY_PLAN.md`):
16 measured PASS, 0 FAIL, 3 observe-only items (2026-10-16 run). The
2026-10-17 example-pattern regression adds T18..T21: 20 PASS / 0 FAIL /
1 OBS, where the pair checks are bounded-average smoke criteria and T20
is register-level math only - per-criterion scope is documented there.
