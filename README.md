# Daidalos — propulsive landing guidance for the RP2350

Flight code for a TVC model rocket that lands itself on a solid motor. It answers
two questions, two hundred times a second:

- **where to point the nozzle** — two TVC servo angles, in 3-D
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
| TVC | two axes, ±10° servo each, 2:1 to the nozzle |
| roll | uncontrolled, 0–90 °/s, either direction |

---

## Repository layout

```
Daidalos_Landing_Guidance/
  Daidalos_Landing_Guidance.ino   I/O shell: pins, timing, link, failsafes
  guidance.h  guidance.cpp        the algorithm — no Arduino dependency
Ariadna_V2/                       three-MCU flight computer built around it
  PINOUT.md                       every pin of the whole avionics, one file
  common/ariadna_link.h           the inter-processor wire protocol
  common/ariadna_kf.h             the navigation Kalman filter
  common/ariadna_uwb.h            UWB multilateration, degrading gracefully
  common/ariadna_rgb.h            the shared LED / buzzer annunciator engine
  Master_MCU/                     Pico 2 W — state machine, servos, log, LoRa
  Position_MCU/                   XIAO RP2350 — sensor fusion, UWB tag
  Navigation_MCU/                 XIAO RP2350 — guidance, TVC, pyros
  Uwb_Anchor/                     XIAO RP2350 ×4 — ground ranging beacons
  Ground_Station/                 Pico 2 — LoRa receiver
  Ground_Visualizer/              PyQt6 telemetry display
test/
  host_test.cpp                   124 numerical checks against the model
  link_test.cpp                   checks on the Ariadna V2 link codec
  est_test.cpp                    checks on the navigation filter
  uwb_test.cpp                    checks on the UWB multilateration
  check_py_format.py              pins the Python decoder to the C packet
  golden_vectors.h                reference values from the simulation
  gen_golden.py                   regenerates them from lander_v2.py
  stub/                           fake Arduino headers, for the syntax check
  Makefile
```

`Daidalos_Landing_Guidance/` is the single-board V1 computer: it takes a navigation
solution in and puts servo commands out. `Ariadna_V2/` splits that job across three
processors and adds the sensors, the mission state machine and the ground segment
around it — flying the same guidance module, unchanged and byte-identical. It also adds
a **UWB constellation**: four DWM1000 anchors on the ground around the pad, fused with
GPS, the barometer and the IMU in the same Kalman filter, which is what takes horizontal
knowledge from GPS's ~2.5 m to a decimetre. See
[`Ariadna_V2/README.md`](Ariadna_V2/README.md) for how each board spends its time, and
[`Ariadna_V2/PINOUT.md`](Ariadna_V2/PINOUT.md) for every pin in the vehicle.

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
| GP2 | TVC servo, **axis 1 (x)** | 333 Hz pulse train |
| GP4 | TVC servo, **axis 2 (y)** | 333 Hz |
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

The fusion processor sends a 61-byte little-endian frame. Anything faster than 200 Hz is
fine — the control loop always uses the newest frame it has.

| offset | size | field | |
|---|---|---|---|
| 0 | 2 | sync | `0xDA 0x1D` |
| 2 | 1 | flags | bit 0 = solution valid |
| 3 | 4 | `h` | m above the pad |
| 7 | 4 | `vx` | m/s, world +x |
| 11 | 4 | `vy` | m/s, world +y |
| 15 | 4 | `vz` | m/s, **negative while descending** |
| 19 | 12 | `bx, by, bz` | **thrust-axis** unit vector, world frame |
| 31 | 12 | `gx, gy, gz` | **body-fixed reference axis**, world frame |
| 43 | 12 | `wx, wy, wz` | rad/s, world frame, **straight off the gyro** |
| 55 | 4 | `t_ms` | uint32 ms, monotonic |
| 59 | 2 | CRC-16/CCITT-FALSE | over bytes 2..58 |

Encoder for the sending side:

```c
static void nav_send(const float pv[4],      // h, vx, vy, vz
                     const float b[3],       // thrust-axis unit vector
                     const float g[3],       // body-fixed reference axis
                     const float w[3],       // rad/s, world frame
                     uint32_t t_ms, bool valid)
{
    uint8_t f[61];
    f[0] = 0xDA; f[1] = 0x1D; f[2] = valid ? 1 : 0;
    memcpy(&f[3],  pv,   16);
    memcpy(&f[19], b,    12);
    memcpy(&f[31], g,    12);
    memcpy(&f[43], w,    12);
    memcpy(&f[55], &t_ms, 4);
    uint16_t crc = crc16_ccitt(&f[2], 57);      // same routine as the .ino
    f[59] = crc & 0xFF; f[60] = crc >> 8;
    NAV_PORT.write(f, sizeof f);
}
```

### Attitude is two unit vectors, not Euler angles

`b` is the direction **thrust pushes the vehicle** — the tail-to-nose axis reversed, so it
points roughly up during the descent. `g` is a second, **body-fixed** axis marking the
airframe's roll orientation: send the body axis that servo 1 deflects the nozzle in. Both come
from the quaternion in one rotation each:

```c
float b[3];  quat_rotate(q, BODY_THRUST_AXIS, b);   // e.g. body -Z
float g[3];  quat_rotate(q, BODY_SERVO1_AXIS, g);   // e.g. body +X
float w[3];  quat_rotate(q, gyro_body, w);          // same q, same frame
```

**`g` is what makes a rolling airframe flyable.** Without it the guidance has no way to know
where the servo axes have got to, and would command them as though the vehicle had never
turned. It need not be exactly perpendicular to `b` — the guidance orthogonalises it — but it
must be body-fixed, and it must be the axis servo 1 really acts in, or the two channels are
mixed by a constant angle for the entire flight.

Three reasons this beats sending angles. It has no convention to get backwards; it has no
wrap and no gimbal lock; and it saves the flight computer two trig calls per tick, which more
than pays for the entire second gimbal axis. If you can only send Euler angles, the loop still
works — it just costs 240 cycles more, which is irrelevant at 0.03 % duty.

Send `b` normalised. The guidance renormalises defensively, and rejects the frame outright if
`|b|` is more than 5 % off or `bz ≤ 0.03` — a vehicle that far over is a link fault, not a
state.

### `w` must be the measured rate

Not `(attitude - previous attitude)/dt`. This is the single most consequential line in the
whole system. Differencing a fused attitude estimate feeds estimator noise into the derivative
term; at 200 Hz, 0.1° of noise becomes ~0.35 rad/s of phantom rate, which after the servo
scaling saturates the actuators. In simulation, under the flight sensor budget, that one choice
is the difference between a **1.3 %** and a **75.6 %** landing rate — from tumbling to flying.
It is worse in 3-D than in the plane, because the noise now enters both channels independently
and the attitude gate is on the total tilt.

The component of `w` along `b` is the roll rate. Send it — the guidance projects it out of the
rate loop, because a two-axis gimbal cannot produce a torque about the body axis and feeding
one in asks for something impossible, and then uses it to cancel the gyroscopic cross-coupling
the spin creates.

### Sign conventions

World frame, z up, origin at the pad. `vx`/`vy` are positive along world +x/+y. Servo **axis 1**
deflects the nozzle in the plane containing the reference axis `g`, **axis 2** in the
perpendicular one. Those are **body-fixed** directions — they turn with the vehicle as it
rolls, which is what the servos physically do.

Get either servo sign backwards and that channel is positive feedback. Test **both** — see the
pre-flight list at the end.

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

- *Attitude*, 200 Hz on core 0, entirely in vector form. The error is `e = b × û` — the
  rotation that closes the gap between the body axis and the commanded thrust direction. That
  is the vector version of the planar `sin(θ_cmd − θ)`: no `atan2`, no wrapping, no gimbal
  lock, and perpendicular to `b` by construction, so the loop can never command a roll it has
  no way to produce. Cascaded outer-attitude-P / inner-rate-P at bandwidth ω_n = 9 rad/s and
  damping ζ = 1.0, with the fin weathercocking torque cancelled by feedforward. Both channels
  share one gain pair — the transverse inertia is identical about either axis.

  The commanded torque becomes a nozzle deflection by **dynamic inversion**,
  `s = (b × τ)/(L·T)`, which is exact rather than a linearisation (round trip verified to
  1e-14). Its components along the two gimbal axes are the two servo commands. Thrust sweeps
  about 230× over a burn; without inverting it the loop gain sweeps with it and no single gain
  pair works throughout.
- *Planner*, core 1. A receding-horizon search for the throttle profile that brings the
  vehicle to rest at the pad. The plan is one scalar `p ∈ [0,4]` that fills four clamp
  segments backwards from burnout — "coast, then slam" — chosen so the projected miss
  distance is monotone in `p`, which makes the root find well posed. Solved by safeguarded
  false position (Illinois), warm-started from the previous answer.

The tilt command itself is the zero-effort-velocity law, `t_go = 2h/|vz|`,
`a_h = −v_h/t_go`, `a_z = |v_z|/t_go + g`, credited with the drag that is already
decelerating the vehicle so it stops over-tilting, and limited to a **cone** of
`min(45°, 3h + 5°)` so it cannot ask for a large tilt near the ground — the same limit in
every azimuth.

**That law stays planar in 3-D, and it is why the second axis is nearly free.** Both
horizontal terms — the ZEV demand and the drag credit — are anti-parallel to the horizontal
velocity, so its direction never changes along a projected trajectory. The 3-D command is
exactly the planar solution rotated into that azimuth, verified as an identity to 2e-16 over
200 000 states. So the trajectory projection stays two-dimensional, the planner is untouched,
and the guidance call costs one extra `sqrtf`.

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
| attitude as a unit vector, error as `b × û` | no Euler angles anywhere in the loop, so no `atan2`, no wrapping, no gimbal lock — and no `sinf`/`cosf` to turn an angle back into a direction |
| `asin(x)/x ≈ 1 + x²/6` on the error vector | one multiply-add rescales the whole vector, instead of three `asinf` calls |
| exact vector dynamic inversion | `s = (b × τ)/(L·T)`; one cross product replaces a per-axis formulation, and it is not a linearisation |
| no malloc, no recursion, no unbounded loops | every loop in the flight path has a compile-time bound |

Measured cost of this structure: **~0.5 % of one core** for the planner plus ~0.03 % for the
attitude loop. The `loop_us` and `overrun` telemetry columns report the real figures on your
board — check them before trusting the estimate.

### What the second gimbal axis costs

Nothing in the expensive half, and *negative* cycles in the cheap half.

The planner is untouched: its trajectory projection stays two-dimensional, for the reason
given under the guidance law. The attitude loop does roughly twice the arithmetic — a gimbal
basis, three cross products, a roll-rate projection, two servo outputs — but drops the two
`sinf`/`cosf` calls the planar version needed to turn a reported angle into a direction,
because the attitude now arrives as a vector. Net: **365 cycles → 215.**

| item | cost of the second axis |
|---|---|
| trajectory projection, planner, root find | zero |
| ignition-altitude table | zero — still 1-D in \|Vz₀\| |
| guidance direction | one extra `sqrtf` |
| attitude loop | −150 cycles |

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
make          # 124 numerical checks against the simulation
make est      # the Ariadna V2 navigation filter
make uwb      # the Ariadna V2 UWB multilateration, degraded cases included
make link     # the Ariadna V2 link codec and transmit ring
make sync     # the V2 copy of the guidance module has not drifted
make pyfmt    # the Python ground station matches the C packet layout
make syntax   # compiles the sketches against stub Arduino headers
make check    # all of it
```

`make` compiles `guidance.cpp` **unchanged** and compares it to golden vectors generated
from the Python model: the motor curve, 72 guidance-tilt cases, 8 root-find states and the
ignition table. On top of that it checks the properties that make the 3-D form correct —

- the 3-D law **reduces exactly** to the planar law when the crossrange channel is zero;
- the command is **isotropic**: rotating the horizontal velocity rotates the command with it;
- the gimbal basis is **orthonormal and perpendicular** to the body axis, and a tilt purely
  toward +y leaves axis 1 on world x, which is what makes the two servos independent;
- the **dynamic inversion round-trips exactly** — the deflection it solves for produces the
  torque that was asked for;
- a **pure roll rate is projected out** and does not move either servo, but is reported;
- the **gimbal axes follow the airframe through a full 360° roll** — a basis anchored to the
  world passes every other check in the file and fails only this one;
- a **sloppy reference axis** (non-unit, not perpendicular to `b`) still yields an orthonormal
  basis, so the sender is allowed to be approximate;

— and the structural invariants: monotonicity of the miss function, servo travel and
quantisation, clamp range, both servos parked during the coast, no commands on invalid nav.

Regenerate the vectors after any change to the model:

```
make golden MODEL=../../Landing-Rocket-Sim/lander_v2.py
```

`gen_golden.py` *imports* the model rather than re-implementing it, so the vectors cannot
drift away from the simulation they came from.

Current status: **124 checks, 0 failures.** Worst guidance-tilt error over 72 cases is
0.00005 rad (0.003°); the reduction, isotropy, orthonormality, inversion and roll-tracking
checks all pass at machine precision.

The golden vectors were **bit-identical before and after the 3-D upgrade**, which is the
regression that matters: it says the planar law, the projection, the solver and the ignition
table all came through the rewrite untouched.

`make syntax` will not catch a wrong pin or an inverted servo — it catches the class of
mistake that otherwise only appears with a board in front of you: a typo, a wrong argument
type, a rename in `guidance.h` the sketch was not updated for.

---

## Telemetry

CSV on USB at 20 Hz:

```
t_ms,phase,h,vx,vy,vz,tilt,tilt_cmd,tilt_err,wmag,roll_dps,servo1,servo2,clamp,thrust,p,
rx_ok,rx_bad,loop_us,overrun,plan
```

`phase` is 0 idle, 1 coast, 2 burn, 3 done. `tilt` is the angle from vertical in degrees and
`tilt_err` the total angle between commanded and actual body axis, so both are azimuth-free.
`roll_dps` is the uncontrolled spin — watch it, but nothing on this board can change it.
`loop_us` is the worst control-loop execution time seen so far and `overrun` counts missed
200 Hz slots — both should stay near zero. `plan` counts planner iterations; if it stops
advancing, core 1 has died.

---

## Expected performance

From 2880 three-dimensional Monte-Carlo scenarios per configuration in the simulation, with
common random numbers. Gates: |vz| < 3 m/s, horizontal **speed** < 0.75 m/s, **tilt** < 10°,
|ω| < 60 °/s at touchdown — each on a magnitude, so they apply in any azimuth.

| entry \|Vz\| at 85 m | design dispersions | full dispersions |
|---|---|---|
| 20–45 m/s | 99.7–100 % | 94–98 % |
| 50 m/s | 92.2 % | 87.5 % |
| 55 m/s | 66.1 % | 62.5 % |

*Design* dispersions are igniter delay U(0.1, 0.3) s plus grain variation (1 % impulse,
2 % burn rate). *Full* adds 3 m/s wind with gusts, mass and Cd spread, 0.30° nozzle
misalignment, and the flight sensor budget. Entry azimuth and wind azimuth are drawn
independently per flight, and **both** profiles include the 0–90 °/s roll.

**Keep entry below ~45 m/s at 85 m.** Above that the vehicle runs out of grain before it
runs out of velocity — a sizing problem, not a guidance one. The ΔV budget makes this
concrete: of ~94 m/s the grain delivers, clamp waste takes 22–41 m/s and gravity 18 m/s,
while **steering costs 0.6 m/s**. Perfect attitude control would recover almost nothing.

**Also keep entry drift below ~12 m/s in wind.** Horizontal entry speed looks free under
design dispersions (93–98 % from 0 to 15 m/s) but falls to **82 %** at 15 m/s under full ones,
because entry and wind azimuths are independent — the vehicle can be asked to kill a large
drift in one direction while the wind pushes it in another.

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
   **The nozzle must swing to oppose the motion — test BOTH axes.** Push the nose toward
   +x and axis 1 must push back; push it toward +y and axis 2 must. If either follows
   instead, that `TVC*_SIGN` is wrong and the vehicle will diverge in under a second.
   Testing one axis and assuming the other is how a sign error reaches the pad.
3b. **Check the roll reference.** Rotate the airframe about its own long axis and watch the
   nav board's `g`: it must turn with the vehicle, and it must be the axis servo 1 really
   deflects in. If `g` is 90° out from the true servo-1 axis the two channels are swapped for
   the whole flight, and every static bench test still passes. Also confirm the roll rate the
   vehicle arrives with is inside 90 °/s — beyond that is untested, and a vehicle
   *accelerating* in roll (a canted fin) is outside the model entirely.
4. Static fire with the clamp servo connected and the TVC locked, to calibrate
   `CLAMP_US_CLOSED` / `CLAMP_US_OPEN` against measured thrust.
5. Only then fly it, over a range, with the usual model-rocketry safety practice for
   the motors involved.
