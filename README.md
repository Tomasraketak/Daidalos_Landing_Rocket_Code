# Daidalos — propulsive landing guidance for the RP2350

Flight code for a TVC model rocket that lands itself on a solid motor. It answers
two questions, two hundred times a second:

- **where to point the nozzle** — the TVC servo angle
- **how much thrust to keep** — the thrust-clamp command

Everything else (sensor fusion, recovery, telemetry downlink) lives elsewhere. This
board takes a navigation solution in and puts two servo commands and one igniter
pulse out.

The algorithm is a port of the one developed and Monte-Carlo tested in
[Landing-Rocket-Sim](https://github.com/Tomasraketak/Landing-Rocket-Sim). It is pinned
to that simulation numerically rather than by inspection — see [Verification](#verification).

---

## The vehicle it assumes

Two solid motors. The ascent motor is aft, by the fins. The **landing** motor sits under
the nose cone pointing up during the climb and down during the descent, because the fins
bring the vehicle over nose-first. Its nozzle gimbals; the arm from the CG to the nozzle
is 0.70 m.

The landing motor cannot be throttled. It has a **thrust-diverting clamp** that splits
exhaust to both sides — no net side force — so commanding less thrust *wastes* impulse
rather than saving it. Two consequences drive the whole design:

- the grain burns for ~1.7 s no matter what the clamp does, so **burn duration is a hard
  constraint**, not a variable;
- impulse not used to decelerate is gone. Igniting too high is recoverable (clamp down);
  igniting too low is not recoverable at all. Every margin in this code is asymmetric for
  that reason.

| | |
|---|---|
| dry / wet mass | 2.87 / 3.20 kg |
| grain | 0.330 kg, 286.8 N·s, ~1.75 s |
| peak thrust | 230 N |
| body | 1.40 m long, 105 mm diameter |
| guidance entry | 85 m altitude |
| entry envelope | \|Vz\| 20–45 m/s (see below), \|Vx\| 0–15 m/s |
| TVC travel | ±10° servo, 2:1 to the nozzle |

---

## Repository layout

```
Daidalos_Landing_Guidance/
  Daidalos_Landing_Guidance.ino   I/O shell: pins, timing, link, failsafes
  guidance.h  guidance.cpp        the algorithm — no Arduino dependency
Ariadna_V2/                       three-MCU flight computer built around it
  common/ariadna_link.h           the inter-processor wire protocol
  Master_MCU/                     Pico 2 W — state machine, servos, log, LoRa
  Position_MCU/                   XIAO RP2350 — sensor fusion
  Navigation_MCU/                 XIAO RP2350 — guidance, TVC, pyros
  Ground_Station/                 Pico 2 — LoRa receiver
  Ground_Visualizer/              PyQt6 telemetry display
test/
  host_test.cpp                   108 numerical checks against the model
  link_test.cpp                   85 checks on the Ariadna V2 link codec
  check_py_format.py              pins the Python decoder to the C packet
  golden_vectors.h                reference values from the simulation
  gen_golden.py                   regenerates them from lander_v2.py
  stub/                           fake Arduino headers, for the syntax check
  Makefile
```

`Daidalos_Landing_Guidance/` is the single-board V1 computer: it takes a navigation
solution in and puts servo commands out. `Ariadna_V2/` splits that job across three
processors and adds the sensors, the mission state machine and the ground segment
around it — flying the same guidance module, unchanged and byte-identical. See
[`Ariadna_V2/README.md`](Ariadna_V2/README.md).

**The algorithm is deliberately not in the `.ino`.** Anything inside a sketch can only be
tested by flashing a board, and firmware that cannot be tested on a workstation ends up
being tested on the pad. `guidance.cpp` includes no Arduino headers, so the host test
compiles the exact file that flies.

---

## Building and flashing

1. Arduino IDE → Board Manager → install **arduino-pico** (earlephilhower).
2. Select **Raspberry Pi Pico 2** (RP2350).
3. Open `Daidalos_Landing_Guidance/Daidalos_Landing_Guidance.ino`.
4. Set CPU speed 150 MHz (default) and upload.

No libraries are needed. Servo pulses come from the RP2350's own PWM slices rather than
the `Servo` library, which refreshes at a fixed 50 Hz — up to 20 ms of dead time in a loop
running at 200 Hz, which is real phase lag at a 9 rad/s bandwidth.

---

## Wiring

| pin | signal | notes |
|---|---|---|
| GP2 | TVC servo | 333 Hz pulse train |
| GP3 | thrust-clamp servo | 333 Hz |
| GP6 | igniter MOSFET gate | LOW = safe; drives a gate, never the igniter directly |
| GP7 | arm switch | LOW = armed, internal pull-up |
| GP8 | UART1 TX → nav board RX | 460800 baud |
| GP9 | UART1 RX ← nav board TX | |
| USB | telemetry, 921600 baud | CSV |

Servos on their own BEC with a common ground. A servo brown-out during the burn is a
crash, and the burn is only 1.7 s long.

---

## The navigation link

The fusion processor sends a 29-byte little-endian frame. Anything faster than 200 Hz is
fine — the control loop always uses the newest frame it has.

| offset | size | field | |
|---|---|---|---|
| 0 | 2 | sync | `0xDA 0x1D` |
| 2 | 1 | flags | bit 0 = solution valid |
| 3 | 4 | `h` | m above the pad |
| 7 | 4 | `vx` | m/s, + downrange |
| 11 | 4 | `vz` | m/s, **negative while descending** |
| 15 | 4 | `theta` | deg from vertical, + leaning toward +x |
| 19 | 4 | `omega` | deg/s, **straight off the gyro** |
| 23 | 4 | `t_ms` | uint32 ms, monotonic |
| 27 | 2 | CRC-16/CCITT-FALSE | over bytes 2..26 |

Encoder for the sending side:

```c
static void nav_send(float h, float vx, float vz, float theta_deg,
                     float omega_dps, uint32_t t_ms, bool valid)
{
    uint8_t b[29];
    b[0] = 0xDA; b[1] = 0x1D; b[2] = valid ? 1 : 0;
    memcpy(&b[3],  &h,         4);
    memcpy(&b[7],  &vx,        4);
    memcpy(&b[11], &vz,        4);
    memcpy(&b[15], &theta_deg, 4);
    memcpy(&b[19], &omega_dps, 4);
    memcpy(&b[23], &t_ms,      4);
    uint16_t crc = crc16_ccitt(&b[2], 25);      // same routine as the .ino
    b[27] = crc & 0xFF; b[28] = crc >> 8;
    NAV_PORT.write(b, sizeof b);
}
```

### `omega` must be the measured rate

Not `(theta - theta_prev)/dt`. This is the single most consequential line in the whole
system. Differencing a fused attitude estimate feeds estimator noise into the derivative
term; at 200 Hz, 0.1° of noise becomes ~0.35 rad/s of phantom rate, which after the servo
scaling saturates the actuator. In simulation, under the flight sensor budget, that one
choice is the difference between a **6.9 %** and a **72.6 %** landing rate — from tumbling
to flying.

### Sign conventions

`theta` is measured from vertical toward +x, and `vx` is positive in the same direction.
A vehicle leaning downrange has positive `theta`. Get this backwards and the attitude loop
becomes positive feedback.

---

## How it flies

**Arm.** The physical switch is closed *and* the vehicle falls through 85 m descending
faster than 2 m/s. The entry velocity latched at that moment selects the ignition
altitude from a table.

**Coast.** No motor, so the gimbal produces no torque and the servo is parked at zero.
Attitude error during the coast is expected and is not a control failure — there is
nothing to control with.

**Ignition** fires on *predicted altitude*, leading by the mean igniter delay, with a pad
sized to the delay **spread**. Firing on a clock instead forces the pad to cover the
worst-case delay for every flight; firing on altitude shrinks it, and that is where the
accuracy at the top of the envelope comes from.

**Burn.** Two loops:

- *Attitude*, 200 Hz on core 0. Cascaded outer-attitude-P / inner-rate-P, parameterised by
  a bandwidth ω_n = 9 rad/s and damping ζ = 1.0, with the fin weathercocking torque
  cancelled by feedforward. The commanded torque is then converted to a nozzle angle by
  **dynamic inversion**, `sin δ = −τ/(T·L)`. Thrust sweeps about 230× over a burn; without
  inverting it the loop gain sweeps with it and no single gain pair works throughout.
- *Planner*, core 1. A receding-horizon search for the throttle profile that brings the
  vehicle to rest at the pad. The plan is one scalar `p ∈ [0,4]` that fills four clamp
  segments backwards from burnout — "coast, then slam" — chosen so the projected miss
  distance is monotone in `p`, which makes the root find well posed. Solved by safeguarded
  false position (Illinois), warm-started from the previous answer.

The tilt command itself is the zero-effort-velocity law, `t_go = 2h/|vz|`,
`a_x = −v_x/t_go`, `a_z = |v_z|/t_go + g`, credited with the drag that is already
decelerating the vehicle so it stops over-tilting, and limited to
`min(45°, 3h + 5°)` so it cannot ask for a large tilt near the ground.

---

## RP2350 specifics

The Cortex-M33 has a **single-precision** FPU. A `double` is emulated in software and runs
10–50× slower, so the whole module is `float` and the host build compiles with
`-Wdouble-promotion` — `guidance.cpp` is clean under it. In the sketch the warning is
suppressed *only* around diagnostic prints, where `printf`'s varargs promote by language
rule; that keeps the warning meaningful everywhere it matters.

| what | why |
|---|---|
| thrust direction carried as a unit vector | removes `atan2f` from the guidance law and `sinf`/`cosf` from the projection inner loop — about 65 % of the cost of a projection step |
| `asin(x) ≈ x + x³/6` | the nozzle never exceeds 5°, where the series is good to 1e-5 rad |
| RK2 at dt = 0.04 s | half the steps of the model's Euler at 0.02 s, same answer to within 0.035 in `p` |
| 64-entry uniform thrust table | O(1) lookup instead of a 13-point scan. It spans [τ_ign, 1]; the 11 ms ignition ramp below that is evaluated exactly, because no uniform grid can resample a near-vertical rise |
| ignition altitudes precomputed | the model's `find_ignition` solve costs ~46 ms and must never run in flight |
| **amortised** root find | one trajectory projection per 200 Hz call, converging in ~35 ms, instead of a whole solve as a multi-millisecond spike every 100 ms |
| no malloc, no recursion, no unbounded loops | every loop in the flight path has a compile-time bound |

Measured cost of this structure: **~0.8 % of one core** for the planner plus ~0.05 % for
the attitude loop. The `loop_us` and `overrun` telemetry columns report the real figures
on your board — check them before trusting the estimate.

### Concurrency

Core 0 writes the navigation state, core 1 reads it, through a seqlock — the 200 Hz loop
must never block on the planner. The one ordering constraint is `S.phase`: it is what
wakes the planner, so the solver is reset *before* the phase becomes visible. Everything
else shared between the cores is a naturally aligned 32-bit word, where a torn read is
impossible and a stale read is harmless.

Set `DG_USE_CORE1` to 0 to run the planner inline on core 0 instead. Same results, one
core, no concurrency at all — a reasonable trade if you would rather not reason about two.

---

## Failsafes

| condition | response |
|---|---|
| arm switch open | igniter output forced low, checked again at the output stage |
| nav gap > 60 ms | dead-reckon forward on thrust and gravity |
| nav gap > 250 ms | state marked invalid: servo parks, **clamp freezes where it is** |
| CRC failure or out-of-envelope frame | dropped, counted in `rx_bad` |
| `valid` flag clear | no ignition, no servo motion |
| self-test failure at boot | fast LED blink, loop never starts |

Freezing the clamp on a lost link is deliberate. Shutting it during a burn guarantees a
crash, and the grain finishes burning either way.

---

## Verification

```
cd test
make          # 108 numerical checks against the simulation
make link     # 107 checks on the Ariadna V2 link codec and transmit ring
make sync     # the V2 copy of the guidance module has not drifted
make pyfmt    # the Python ground station matches the C packet layout
make syntax   # compiles the sketches against stub Arduino headers
make check    # all of it
```

`make` compiles `guidance.cpp` **unchanged** and compares it to golden vectors generated
from the Python model: the motor curve, 72 guidance-tilt cases, 8 root-find states, the
ignition table, and structural invariants (monotonicity of the miss function, servo travel
and quantisation, clamp range, servo parked during the coast, no commands on invalid nav).

Regenerate the vectors after any change to the model:

```
make golden MODEL=../../Landing-Rocket-Sim/lander_v2.py
```

`gen_golden.py` *imports* the model rather than re-implementing it, so the vectors cannot
drift away from the simulation they came from.

Current status: **108 checks, 0 failures.** Worst guidance-tilt error over 72 cases is
0.00005 rad (0.003°).

`make syntax` will not catch a wrong pin or an inverted servo — it catches the class of
mistake that otherwise only appears with a board in front of you: a typo, a wrong argument
type, a rename in `guidance.h` the sketch was not updated for.

---

## Telemetry

CSV on USB at 20 Hz:

```
t_ms,phase,h,vx,vz,theta,omega,theta_cmd,err,servo,clamp,thrust,p,rx_ok,rx_bad,loop_us,overrun,plan
```

`phase` is 0 idle, 1 coast, 2 burn, 3 done. `loop_us` is the worst control-loop execution
time seen so far and `overrun` counts missed 200 Hz slots — both should stay near zero.
`plan` counts planner iterations; if it stops advancing, core 1 has died.

---

## Expected performance

From 2880 Monte-Carlo scenarios per configuration in the simulation, with common random
numbers. Gates: |vz| < 3 m/s, |vx| < 0.75 m/s, |θ| < 10°, |ω| < 60 °/s at touchdown.

| entry \|Vz\| at 85 m | design dispersions | full dispersions |
|---|---|---|
| 20–45 m/s | 99.4–100 % | 87–94 % |
| 50 m/s | 88.9 % | 83.6 % |
| 55 m/s | 72.5 % | 69.4 % |

*Design* dispersions are igniter delay U(0.1, 0.3) s plus grain variation (1 % impulse,
2 % burn rate). *Full* adds 3 m/s wind with gusts, mass and Cd spread, 0.30° nozzle
misalignment, and the flight sensor budget.

**Keep entry below ~45 m/s at 85 m.** Above that the vehicle runs out of grain before it
runs out of velocity — a sizing problem, not a guidance one. The ΔV budget makes this
concrete: of ~94 m/s the grain delivers, clamp waste takes 22–41 m/s and gravity 17 m/s,
while **steering costs 0.6 m/s**. Perfect attitude control would recover almost nothing.

---

## Before you fly this

This software is **unflown**. It is verified in simulation and by numerical equivalence
against that simulation. It has not been through hardware-in-the-loop testing or a flight,
and no amount of passing host tests substitutes for either.

The pin map, servo travel, pulse calibration, igniter timing, packet format and sign
conventions in this repository describe an *intended* vehicle. Check every one against
yours.

Suggested order:

1. Bench, no motor, `PIN_ARM` open. Confirm the self-test passes and telemetry flows.
2. Feed recorded or synthetic nav frames. Confirm ignition triggers at the expected
   altitude and that `loop_us` and `overrun` stay where you expect.
3. Deflect the airframe by hand with the loop running and a dummy thrust estimate.
   **The nozzle must swing to oppose the motion.** If it follows instead, `TVC_SIGN` is
   wrong and the vehicle will diverge in under a second.
4. Static fire with the clamp servo connected and the TVC locked, to calibrate
   `CLAMP_US_CLOSED` / `CLAMP_US_OPEN` against measured thrust.
5. Only then fly it, over a range, with the usual model-rocketry safety practice for
   the motors involved.
