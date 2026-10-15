# Ariadna V2 — flight computer for Daidalos

Three microcontrollers, one ground station, one laptop. This directory is the
structural foundation: it compiles, it is tested where it can be tested, and it is
written to be finished rather than admired.

The guidance algorithm it flies is not new — it is the verified module in
[`../Daidalos_Landing_Guidance`](../Daidalos_Landing_Guidance), unchanged. What is new
is everything around it: three boards, a wire protocol, sensor fusion, a mission state
machine, and the ground segment.

---

## The split

| board | hardware | owns |
|---|---|---|
| **Master** | Raspberry Pi Pico 2 W | mission state machine, all servos, flash log, LoRa downlink, arming |
| **Position** | XIAO RP2350 | sensor fusion, the navigation solution, the UWB tag |
| **Navigation** | XIAO RP2350 | guidance, ignition timing, TVC, pyros |
| **Ground station** | Raspberry Pi Pico 2 | LoRa receive, OLED, USB uplink to the laptop |
| **UWB anchor** ×4 | XIAO RP2350 | fixed ranging beacons on the ground around the pad |

One rule decides which board does what: **nothing that can block may share a core with
something that must not.** A barometer read blocks on I²C. A flash sector erase blocks for
up to 400 ms. A LoRa packet is 92 ms on air. None of those can sit next to a 200 Hz
control loop, so none of them do.

```
   Position MCU                Master MCU                 Navigation MCU
  ┌────────────┐             ┌────────────┐              ┌────────────┐
  │ TriSense   │  nav state  │  routing   │  nav state   │  guidance  │
  │ GPS LK76   │ ──240 Hz──► │   state    │ ──240 Hz──►  │  planner   │
  │ estimator  │             │  machine   │              │  pyros     │
  │            │             │            │ ◄──200 Hz──  │  TVC solve │
  └────────────┘             │  PCA9685   │  actuators   └────────────┘
                             │  W25Q128   │
                             │  LoRa      │
                             └─────┬──────┘
                                   │ 433 MHz, 10 Hz
                             ┌─────▼──────┐      USB      ┌────────────┐
                             │ Ground stn │ ──framed────► │ visualiser │
                             └────────────┘               └────────────┘

   and on the ground, around the pad:

     ┌────────┐   ┌────────┐        UWB two-way ranging, ~10 Hz per anchor
     │anchor 1│   │anchor 2│   ◄─────────────────────────────────┐
     └────────┘   └────────┘                                     │
     ┌────────┐   ┌────────┐                            the tag on the
     │anchor 4│   │anchor 3│   ◄──────────────────────  Position MCU
     └────────┘   └────────┘
```

---

## What each MCU actually does, and how

The table above says who owns what. This section says what each board *does with
its time* — the loops, the cores, the rates, and the one thing on each board that
must never be allowed to block.

Read it before changing anything on a board you have not worked on: almost every
non-obvious decision in this firmware is a consequence of a timing constraint that
is invisible from inside a single file.

### Position MCU — *where am I?*

**Hardware:** XIAO RP2350. ICM42688P over SPI, BMP580 + AK09918C over I²C (all three
on the Voltino TriSense), LK76 GPS on a PIO UART, DWM1000 UWB on the same SPI bus as
the IMU, TF Luna lidar on the same I²C bus (declared, not implemented).

**Product:** exactly one thing — `ariadna_nav_state_t`, 88 bytes, emitted at 240 Hz.
It makes no control decisions whatsoever.

| core | runs | may block? |
|---|---|---|
| 0 | *all* sensor I/O, the estimator, the UWB tag, every outgoing packet | no |
| 1 | GPS NMEA parsing, receiving from the Master | yes, freely |

Every I²C and SPI transaction in the vehicle's sensor stack happens on **core 0**, so
there is no bus arbitration to get wrong and the DMA transmit ring has exactly one
producer. Core 1 touches no shared bus, which is what lets it be as bursty as NMEA
wants to be. The GPS fix crosses between them through a seqlock; neither core ever
waits on the other.

One estimator tick, in order:

1. **service the UWB radio and the LED** — in the *idle path*, before the tick-rate
   test, so ranging throughput is set by the core's spare time rather than by the
   estimator's period;
2. **drain the IMU FIFO** through the fusion library (240 Hz);
3. **read the barometer** — but only every 20 ms, and the resulting correction is
   applied on exactly one tick per sample. Correcting on every tick would feed the
   same measurement in five times and make the filter √5 over-confident;
4. **rotate** body-frame linear acceleration and gyro rates into the pad frame with
   this file's own rotation matrix;
5. **predict** all three Kalman axes;
6. **correct** from whichever of the barometer, GPS, UWB and the zero-velocity
   update have something new to say;
7. **build and transmit** the navigation state — immediately, in the same tick that
   produced it, because a buffered state has a latency that varies packet to packet,
   which is worse for a control loop than latency that is merely large;
8. **choose the LED colour** from the flags it just transmitted.

Twenty times a second it also sends `ariadna_pos_status_t`: attitude in degrees,
lat/lon, temperature, filter σ's and UWB constellation health. None of that is on
the 240 Hz path, and that is what makes the 240 Hz path affordable.

### Navigation MCU — *when do I light the motor, and where do I point it?*

**Hardware:** XIAO RP2350. Three pyro channels and three continuity senses. **No
sensors at all** — everything it knows arrives as `nav_state` from the Position MCU,
routed through the Master.

**Product:** `ariadna_actuator_t` at 200 Hz — two TVC servo angles and a thrust
clamp — plus the decision to fire a pyro.

| core | runs | period |
|---|---|---|
| 0 | link, attitude loop, ignition logic, pyro timing, actuator TX | 5 ms, hard |
| 1 | trajectory replanner, **one solver iteration per call** | best effort |

The planner is the interesting part. A full trajectory solve is about seven
projections; running it to convergence inside the control loop would blow the 5 ms
budget outright. So `dg_step_planner()` does exactly one projection per call and
keeps the root find's bracket between calls, converging in ~35 ms of wall time
without ever blocking for a whole solve. The two cores share the planar navigation
state through a seqlock — never a mutex, because core 0 must not be able to wait.

Its link supervision is a three-stage policy, and each stage is a deliberate choice
about how to fail:

| state age | what happens |
|---|---|
| < 60 ms | normal: the state is used as received |
| 60–250 ms | **dead reckoned** forward from the last state and the thrust estimate |
| > 250 ms | the state is marked invalid: TVC parks, and the clamp **freezes** |

The clamp freezes rather than closing because shutting it mid-burn guarantees a
crash, and the grain finishes burning either way.

### Master MCU — *what is the mission doing?*

**Hardware:** Pico 2 W. PCA9685 servo driver, W25Q128 flash, LoRa SX1278, ACS712
current sense, battery divider, RBF pin, breakwire, SK6812 and buzzer.

**Product:** the mission state, the servo pulses, the flash log, and the downlink.

| core | runs | may block? |
|---|---|---|
| 0 | UART routing, PCA9685 writes, the state machine, ADC | no |
| 1 | flash log writes, LoRa transmission | yes — both do |

This split is the clearest example of the one rule. A flash sector erase blocks for
up to 400 ms and a LoRa packet is tens of milliseconds on air; either one inside the
servo path would be a visible glitch in the nozzle. Core 0 pushes log records into a
ring and core 1 drains it.

Core 0's most important property is what it does *not* do: between reading an
actuator packet and writing the PCA9685 it does nothing at all. The nozzle command
already crosses a UART and an I²C bus, and every microsecond added there is phase
lag on a 9 rad/s loop.

It is also the only board that can arm the vehicle, and the only one that decides
what mission state everything else is in — which is why the Position MCU's ZUPT and
its barometer trust are both gated on a `master_state` packet rather than on
anything either board could work out for itself.

### Ground station — *tell me, and let me stop it*

**Hardware:** Pico 2. LoRa, SSD1306 OLED, three buttons.

| core | runs |
|---|---|
| 0 | LoRa receive and the framed USB uplink to the laptop — nothing that blocks |
| 1 | the OLED (23 ms per full redraw) and the buttons |

The OLED is on core 1 for exactly the reason its redraw time suggests. The station
shows enough to run the flight if the laptop dies.

### UWB anchors — *and here is where the pad is*

**Hardware:** XIAO RP2350 and a DWM1000, four of them, standing on the ground.

One loop, no cores to divide: poll the IRQ line, service the ranging stack, throttle,
show a colour. They are the only part of the system that is not bolted to the
vehicle, and the firmware on the vehicle is written on the assumption that they will
fail — see below.

---

## The wire protocol

One header, [`common/ariadna_link.h`](common/ariadna_link.h), defines every packet in the
system. It has no Arduino dependency, so the codec is tested on a workstation rather than
on the pad.

```
0  1  SYNC0  0xA5
1  1  SYNC1  0x1D
2  1  TYPE
3  1  LEN            payload length, 0..96
4  N  payload
4+N 2 CRC-16/CCITT-FALSE over bytes 2..3+N
```

The CRC covers TYPE and LEN as well as the payload, so a corrupted length byte cannot make
a receiver accept a mis-sized frame. It is bit-identical to the routine already flying in
`Daidalos_Landing_Guidance.ino` — there is exactly one checksum in this vehicle.

| type | payload | direction | rate |
|---|---|---|---|
| `NAV_STATE` | 52 B | Position → Master → Navigation | 240 Hz, event-driven |
| `ACTUATOR` | 36 B | Navigation → Master | 200 Hz |
| `NAV_STATUS` | 48 B | Navigation → Master | 20 Hz |
| `POS_STATUS` | 60 B | Position → Master | 20 Hz |
| `MASTER_STATE` | 16 B | Master → both | 10 Hz |
| `CMD` | 8 B | Master → both | on event |
| `TELEMETRY` | 46 B | Master → LoRa → ground | 10 Hz |

Every inter-MCU struct is laid out 4-byte fields first, then 2-byte, then 1-byte, padded to
a multiple of four. The packed and natural layouts are then identical, no member is ever
misaligned, and a `memcpy` of the struct *is* the wire format. `static_assert` catches the
day someone inserts a field and forgets the padding.

### Link speed

**The inter-MCU links run at 921600, not 115200.** They are short PCB traces, so the higher
rate costs nothing, and it keeps each stream under a quarter of the link:

```
Position→Master   nav state @240 Hz + status : 26.1 %
Master→Navigation nav state @240 Hz          : 24.5 %
Navigation→Master actuator @200 Hz + status  : 10.3 %
```

It was 460800 until `nav_state` grew to carry the attitude quaternion, local position and
world-frame acceleration. At 88 bytes that stream costs 49 % of a 460800 link, in each
direction, on two links — enough that one burst of status traffic starts delaying a control
packet. Doubling the rate put it back where the design wants it. The RP2350's UART divisor
hits 921600 exactly at the default 150 MHz clock, so nothing is being rounded.

At 115200 this traffic does not fit at all. One `#define` at the top of `ariadna_link.h`
changes the rate; `make link` prints the resulting budget and fails past half the link.

Debug consoles on the three flight boards are 115200, as is the ground station's USB link
to the laptop. Note that the flight boards' `Serial` is **native USB CDC, not a UART** — the
baud argument is ignored and the link runs at USB speed regardless. It is set for
consistency with every other serial tool here, not because it does anything.

### The navigation state stream

`nav_state` is **not sent on a timer.** The Position MCU emits it the instant its
estimator finishes a step. Two things were wrong with a timer:

- it samples the estimator at an unrelated rate, so the state sits in a buffer for
  anywhere from zero to a full transmit period before it goes out — *variable* latency,
  which is worse for a control loop than latency that is merely large;
- two free-running timers beat against each other, so the age of the state the Navigation
  MCU sees wanders.

The rate is therefore the estimator's rate: **240 Hz, feeding a 200 Hz control loop.**
Deliberately faster than the consumer — at equal rates two crystals drift and a tick
occasionally finds no new frame. At 1.2 frames per tick, every tick has one. `make link`
asserts that the producer outruns the consumer.

#### What it carries

| field | | |
|---|---|---|
| `px py pz` | m | position in pad-ENU. `pz` is height above the pad |
| `vx vy vz` | m/s | velocity, same frame, `vz` positive up |
| `ax ay az` | m/s² | acceleration, same frame, **gravity and estimated bias removed** |
| `qw qx qy qz` | — | body→pad attitude quaternion, normalised |
| `ux uy uz` | — | thrust-axis unit vector, pad-ENU |
| `wx wy wz` | rad/s | raw gyro rotated into pad-ENU |
| `accel_mag` | g | \|a\| including gravity — how the Navigation MCU knows the motor lit |

88 bytes. Attitude in degrees, latitude, longitude, GPS altitude, temperature and the
filter's own accuracy estimates live in `POS_STATUS` at 20 Hz instead, because no control
law reads them and this packet crosses the link *twice*, Position to Master and Master to
Navigation. **Before adding a field to `nav_state`, ask whether a control law reads it.**

Attitude travels twice, as both `q` and `u`, and that is deliberate. `u` is just `q`
applied to the body thrust axis — but it is what the 200 Hz control path consumes, so
carrying it saves that loop a quaternion rotation every tick. Both are derived from the
same rotation in the same step, so they cannot drift apart; `make est` and the boot
self-test both check it.

### The estimator

Three 3-state Kalman filters, one per pad-ENU axis, each holding **position, velocity and
accelerometer bias**. Corrected by the barometer (vertical), GPS position and velocity
(horizontal), and zero-velocity updates on the pad.

Three filters rather than one 9-state filter is an **exact factorisation, not an
approximation**: attitude is supplied externally by the fusion library, so the system matrix
is block diagonal per axis, the process noise is diagonal, and every measurement touches
exactly one axis. A block-diagonal covariance stays block diagonal forever. Every
measurement is scalar and observes one state directly, so H is a unit row and the update
collapses to a handful of multiply-adds — **there is no matrix inversion anywhere in the
navigation code.** `make est` pins the independence property so a future "optimisation"
that shares state between axes fails on a workstation.

#### What it replaced, and why

A third-order complementary filter on the vertical channel, plus dead reckoning with a
fixed 0.15 gain toward GPS velocity and a 20 s leak toward zero on the horizontal. Four
things it could not do:

1. **It had no horizontal position state at all.** Not a tuning weakness — the state did
   not exist, so the `px`/`py` this packet now carries could not have come from it.
2. **Fixed gains cannot weight a measurement by how good it is.** GPS horizontal accuracy
   swings by an order of magnitude with HDOP and satellite count; one gain is right at one
   HDOP and wrong at every other. A Kalman gain is recomputed from the measurement's
   variance every time, which is the entire reason to pay for the covariance.
3. **No outlier rejection**, so one multipath fix went straight into the state. There is
   now a 4σ innovation gate, with a re-anchoring escape hatch for when the *filter*, not
   the sensor, is the thing that is wrong.
4. **The leak toward zero was a documented lie**, and it biased velocity low by roughly
   40 % over a ten-second descent — the one horizontal quantity the guidance law flies.

What was kept: pad-referenced barometry (the ratio form, so pad temperature and absolute
pressure both cancel), a bias state, and dropping trust in the barometer during the burn.

#### The plume is an R, not a bandwidth

The landing motor's plume disturbs static pressure around the airframe, so the barometric
altitude is wrong by metres exactly when altitude matters most. The *sign* depends on where
the static port sits relative to the recirculation and is not worth asserting from a desk;
the magnitude, and the fact that it is transient, are certain. The old filter dropped its
barometer bandwidth during the burn. This one inflates the barometer's measurement variance
instead — and not knowing the sign is itself an argument for doing it that way, because a
variance says *this measurement is bad*, which is true either way, while a correction has to
commit to a direction and doubles the error if it commits wrongly.

Same intent, better mechanism. Inflating R states the thing that is actually true — *this
measurement is now bad* — and lets the filter work out the consequences, including how much
to trust the altitude it already has. Lowering a bandwidth asserts the answer rather than
the evidence, and does nothing to stop a plume-corrupted sample being gated in as good.

#### ZUPT is pre-launch only, and that is a safety property

Standing still is the only condition under which accelerometer bias is *directly
observable*, so the minutes on the pad are where the bias states are actually learned. With
only position measurements, a constant bias and a slowly growing velocity explain a position
error almost equally well — the same convergence takes over two minutes instead of twenty
seconds.

It is also the most dangerous thing in the filter, which is why it is gated on the Master's
mission state and not on the motion test alone:

> ⚠️ A vehicle in steady-state descent at terminal velocity — drag balancing weight — reads
> **exactly 1 g with a quiet gyro.** That is indistinguishable from sitting on the pad by
> any test this board can make from its own sensors. A ZUPT there would pin velocity to
> zero during the descent; the guidance law divides by vertical speed, so it would compute
> an infinite time-to-go and never light the motor.

With no Master state heard from, the answer is no.

#### Losing GPS

The horizontal channel dead reckons, and its covariance grows honestly — `sigma_pos_h` in
`POS_STATUS` is the filter's real confidence, and `quality` in `nav_state` is derived from
it rather than counted up from boot as it used to be.

A **weak speed prior** (σ = 50 m/s, applied at 1 Hz while GPS is stale) replaces the old
leak. Because it goes in as a measurement, the Kalman gain decides how much notice to take:
converged, K ≈ 1/2500 and a real 12 m/s survives ten seconds of it essentially untouched;
after a long outage, K approaches ½ and it holds the estimate inside the envelope. Silent
when the filter knows what it is doing, firm when it does not.

The failure it prevents is not subtle. An unbounded horizontal velocity eventually exceeds
the Navigation MCU's sanity limits, at which point that board rejects the **whole**
`nav_state` — throwing away a perfectly good altitude and attitude and parking the TVC.

#### UWB — the reason it can land *on* the pad

Four DWM1000 anchors stand on the ground around the pad; the Position MCU carries the
tag. Each anchor answers a two-way ranging exchange at about 10 Hz, and
[`common/ariadna_uwb.h`](common/ariadna_uwb.h) turns the four distances into a
position in the pad frame with an honest per-axis 1σ — which the Kalman filter then
consumes exactly as it consumes a GPS fix.

The argument for the whole subsystem is one number. GPS gives horizontal position to
about 2.5 m in good conditions; UWB two-way ranging gives 10–30 cm. Two and a half
metres is the difference between landing on the pad and landing next to it, and no
amount of guidance authority recovers a position error the estimator does not know it
has.

**The interesting problem is not solving four ranges. It is what to do when there are
not four.** Anchors run off their own batteries, sit in the grass, and are
line-of-sight devices pointed at something that is about to light a rocket motor
between them. So the solver degrades in defined steps rather than failing:

| anchors | what comes out | why |
|---|---|---|
| **4+** | full 3-D position | the vertical is published only if the geometry actually observed it |
| **3** | 3-D attempted, 2-D if the vertical is not observable | ground anchors are coplanar: height is mirror-ambiguous and weakly seen near the pad |
| **2** | horizontal, height held from the filter's own estimate | two spheres meet in a circle; Gauss-Newton from the prior takes the nearer point, and the covariance comes out stretched along the ambiguous direction |
| **1** | a line-of-sight scalar, projected onto the axes | a single range is not a position, but it still constrains the distance to that anchor — enough to stop horizontal drift |
| **0** | nothing | the navigation solution carries on exactly as it did before UWB existed |

Two design decisions make that table safe rather than merely clever.

**Every mode produces the same shape of answer: a position plus a per-axis σ, with a
`use_x` / `use_y` / `use_z` flag.** The filter is three independent per-axis Kalman
filters, and a scalar update of one state is all it can accept. Expressing even the
single-anchor case that way means the fusion code in the sketch has *one* path, not
five — there is no mode logic in the estimator to get wrong. A four-anchor fix and a
one-anchor hint differ entirely in their sigmas.

**The sigmas come from the covariance, so a direction the geometry cannot see gets an
enormous σ automatically.** They are the square roots of the diagonal of the inverse
information matrix, and the Cholesky factorisation that produces them *is* the
degeneracy test: if a pivot is not comfortably positive, the geometry does not support
that many unknowns and an axis is dropped rather than divided by something near zero.
`make uwb` checks the calibration by Monte Carlo — 600 noisy trials, and the observed
spread must match the quoted σ.

Everything a bad measurement could do is stopped before it reaches the filter:

* **per-anchor slots with their own timestamps.** An anchor that stops answering ages
  out on its own; nothing has to detect it, and the others carry on unchanged.
* **screening** — non-finite, negative, absurd or implausibly-jumping ranges never
  reach the solver.
* **a post-fit residual test.** A reflected path is always *longer*; ten metres of
  extra path on one anchor cannot be absorbed by a three-unknown fit of four
  equations, so the residual gives it away and the whole fix is dropped. The threshold
  is set so that a 30 cm antenna-delay error — a real, common thing — is still
  accepted. A gate that fires on real data is worse than no gate.
* **a σ floor** in the estimator, covering what the solver's covariance cannot know
  about: anchor survey error, antenna delay calibration, and the offset between the
  tag's antenna and the vehicle's centre of mass.
* **a 4σ innovation gate with re-anchoring**, shorter than the GPS one — because if
  UWB and the filter disagree persistently, UWB is the one to believe.

> ⚠️ **The anchor coordinates are an input, not an output.** Nothing discovers that an
> anchor was moved two metres from where `s_uwb_anchors[]` says it is; the solver will
> produce a position that is confidently wrong by about that much, and the filter will
> prefer it to GPS because it arrives with a small σ. Measure the square with a tape.
> The post-fit residual in `POS_STATUS` is the check on that work: a survey error shows
> up as a residual that will not go below a few tens of centimetres however good the
> radio conditions are.

The vertical channel is deliberately *not* re-anchored from UWB. The barometer is the
vertical's absolute reference and has its own re-anchor; giving two sensors authority
over one state is how a filter ends up oscillating between two answers.

`ARIADNA_NAV_UWB_OK` and `ARIADNA_NAV_UWB_AIDED` are separate flags on purpose. The
first says the radio is ranging; the second says the filter *accepted* what it
produced. A constellation can be perfectly alive and entirely gated out, and the
difference between those two states is exactly what a survey error looks like from the
pad — which is why it is also on the downlink, in the one-byte `uwb` field.

#### A moving vehicle: why the ranges are de-rotated before they are solved

The anchors do not answer at the same instant. They answer one at a time, so a full set of
ranges is smeared over a round-robin period — and least squares then fits a *single*
position to four measurements taken from four different places.

On a stationary tag that costs nothing, which is why almost every UWB example ignores it.
Measured against this solver on a 40 m square with a 200 ms spread:

| descent speed | position error, uncorrected | post-fit residual | outcome |
|---|---|---|---|
| 0 m/s | 0.00 m | 0.00 m | fine |
| 10 m/s | 1.46 m | 0.41 m | **accepted** |
| 20 m/s | 2.08 m | 0.62 m | **accepted** |
| 50 m/s | 11.58 m | 2.14 m | gated out |

**The 50 m/s row is the harmless one.** The residual test catches it and the fix is thrown
away. The dangerous rows are 10 and 20 m/s — the speeds this vehicle has in its last ten and
five metres — where the error is metres, systematically biased *along the direction of
travel*, and small enough in residual to pass every gate. That is the worst kind of sensor
error: confident, wrong, biased the way the vehicle is moving, at the altitude where the
guidance law has the least time to recover.

`ariadna_uwb_derotate()` removes it. A range measured at `t = now − age` is a distance from
where the vehicle *was*; to first order the distance from where it is now is

```
r(now) = r(t) + e · v · age
```

with `e` the unit line of sight and `v` the filter's own velocity. Borrowing the *derivative*
from the filter to correct a position measurement over a fraction of a second is not
circular — a 0.5 m/s velocity error over 200 ms is 0.1 m, an order below the ranging noise.
What the first-order correction leaves behind is ~½·a·age², and that is not ignored: it is
added to the range's sigma, so an old range is both shifted *and* trusted less.

With it, the same descent profile comes out as:

| height | speed | uncorrected | **corrected** |
|---|---|---|---|
| 60 m | 50 m/s | 11.58 m | **0.13 m** |
| 30 m | 35 m/s | 5.01 m | **0.18 m** |
| 10 m | 20 m/s | 2.08 m | **0.22 m** |
| 5 m | 10 m/s | 0.93 m | **0.12 m** |
| 2 m | 5 m/s | 0.31 m | **0.00 m** |

The same reasoning fixed the driver's outlier test, which was a fixed 15 m jump threshold. At
50 m/s a range to one anchor legitimately changes by 5 m between exchanges and 25 m across a
half-second gap, so a fixed threshold starts rejecting the fastest part of the descent — it
looks most like a fault exactly when the data is most real. The allowance now scales with the
vehicle's own speed and the size of the gap.

#### The vertical: anchored below 10 m, and a barometer that gets calibrated

Four anchors on flat ground see height through the direction cosine `h/r`, which collapses as
the vehicle lands. The solver's covariance tracks that honestly, so the height simply stops
being published when the geometry stops supporting it:

| height | σ_z | published? | horizontal σ |
|---|---|---|---|
| 30 m | 0.17 m | yes | 0.26 m |
| 10 m | 0.37 m | yes | 0.19 m |
| 5 m | 0.72 m | yes | 0.18 m |
| 2 m | 1.77 m | yes | 0.18 m |
| 1 m | — | **no** | 0.18 m |
| 0.3 m | — | **no** | 0.18 m |

Note the last column: the *horizontal* accuracy gets **better** all the way down. That is the
geometry working in the vehicle's favour exactly where it matters.

A UWB height is accepted only from a **full four-anchor** constellation — three coplanar
anchors give a horizontal fix worth having and a vertical one that is not — and below 10 m it
goes in with its sigma scaled up. That is the "light anchoring": enough to correct a standing
barometer error over about a second, not enough for a marginal-geometry height to move the
altitude the guidance law is flying in one step. The vertical is still never *re-anchored*
from UWB; the barometer keeps that authority.

**But fusing the two only averages a barometer bias, it does not remove it.** Weighted by
their variances and rates, a metre of offset comes out as most of a metre of altitude error.
Removing it needs the offset estimated as its own quantity, which is what
`ariadna_baro_trim_t` in `ariadna_kf.h` does: a slow (τ = 2.5 s), clamped (±1.5 m), heavily
gated first-order observer of `h_baro − z_uwb`, subtracted from every barometer reading.

Measured end to end through the real estimator, on a 150 m descent at `v ≈ −2.24·√h`
(27 m/s at entry, 20 at 80 m, 10 at 20 m, 5 at 5 m), altitude error below 15 m:

| case | RMS | at touchdown | learned trim |
|---|---|---|---|
| no bias, no UWB — reference | 0.01 m | 0.00 m | — |
| **+1.0 m barometer bias, no UWB** | 0.99 m | **+1.00 m** | — |
| **+1.0 m barometer bias, UWB on** | 0.07 m | **+0.07 m** | +0.93 m |
| −0.8 m bias, UWB on | 0.03 m | −0.03 m | −0.78 m |
| +1.0 m bias, **UWB dies below 2 m** | 0.07 m | **+0.07 m** | +0.93 m |
| +1.0 m bias, **plume below 30 m** | 0.34 m | +1.02 m | +0.88 m |

Three of those rows are the design:

* **93 % of the bias is removed**, both signs.
* **UWB dying below 2 m costs nothing**, because the trim *holds* its learned value. That is
  the point rather than a side effect: the anchor geometry gives up on height in the last two
  metres, so the vehicle lands on a barometer that UWB already calibrated.
* **The plume row is the honest limit.** During the burn the trim is *frozen* — the
  difference between the two heights is mostly plume, not bias, and learning then would store
  a transient as a permanent offset and apply it after burnout. Frozen, the last two metres
  fall back on a plume-corrupted barometer and the error goes back to about a metre. The
  sensor for that regime is the TF Luna lidar, which is wired into the pipeline and not yet
  implemented. **A single anchor on a 3 m mast would also fix it** — it keeps the vertical
  direction cosine alive to the ground, and the anchor table has eight slots and takes any
  `z`.

> ⚠️ **What the trim cannot do.** It cannot tell a barometer bias from a mis-surveyed anchor
> plane. Anchors that really sit 0.5 m from where `s_uwb_anchors[]` says produce a 0.5 m
> altitude error and a trim of ∓0.5 m — measured, not argued. This is why the trim is
> *clamped* rather than unbounded: a survey error shows up as a value pinned at its limit,
> reported in `POS_STATUS` and flashed by the Position MCU's LED as **yellow ×3**, instead of
> being quietly absorbed. Watch it on the pad.

#### Predict at 240 Hz, correct at 50 Hz

The estimator's *prediction* step runs every tick. The barometer *correction* runs only on
a genuinely new sample — about 50 Hz, which is what the BMP580 delivers at the OSR ×16 this
build uses for its altitude noise floor.

That distinction matters. Applying the correction on every predict step would feed the same
measurement in roughly five times, multiplying the effective gain by five and moving the
filter's crossover well above the bandwidth it was tuned for. It would still converge — it
would just be a filter that is roughly √5 more confident in its altitude than the sensor
justifies — and one that then quietly rejects the GPS corrections that disagree with it.

Sampling the barometer faster would not help: a quieter sample at 50 Hz beats a noisier one
at 240 Hz here, and the filter weights it by its variance either way.

### DMA, and where it is deliberately absent

The processor is not short of cycles. CRC and frame parsing together cost about **1 % of a
core** at full traffic — measured, not assumed, and printed by `make link`.

What *was* expensive is different: the UART's hardware transmit FIFO is 32 bytes, and a byte
takes 10.9 µs at 921600 baud. A frame longer than the FIFO cannot be handed over in one go.

| | bytes past the FIFO | per frame | duty |
|---|---|---|---|
| Master forwards `nav_state` @240 Hz | 62 | 673 µs | **16.2 %** of a core |
| Position sends `nav_state` @240 Hz | 62 | 673 µs | **16.2 %** of a core |
| Navigation sends `actuator` @200 Hz | 10 | 109 µs | **2.2 %** of a core |

A blocking write spends that time spinning. Worse than the wasted cycles, on the Master
those 673 µs land *inside the forwarding path* — forwarding the frame before parsing it
saves microseconds, and then blocking on the FIFO gives it all back.

So transmit goes through a ring drained by DMA. Queuing a frame is a memcpy; the processor
never waits for the wire. One channel per port, four in the vehicle.

**DMA is not used anywhere else**, because nothing else is expensive enough to justify
another channel to misconfigure:

| | cost | verdict |
|---|---|---|
| flash logging | 33 µs of SPI, 17×/s, on an idle core 1 | 0.06 % — no |
| LoRa | 46 B at 4 Hz; the 92 ms of airtime dominates | no |
| ADC | 3 blocking conversions ≈ 9 µs per tick, and the IIR already averages ~50 samples | no |
| UART receive | a receive FIFO does not block the processor; unread bytes just wait | no |

The **ring bookkeeping is host-tested** — byte-exact ordering across repeated wraps,
whole-frame drops when full, and proof that the in-flight region cannot be overwritten by a
producer running flat out. The **DMA channel configuration is not**, and cannot be without
the chip. `#define ARIADNA_USE_DMA_TX 0` falls back to blocking writes if it misbehaves on
the bench.

### The downlink, and what makes 10 Hz work

Three separate things had to change for a 10 Hz downlink to be *reliable* rather than
merely configured.

**1. Airtime.** A 47-byte payload at SF7 is 92.4 ms on air at 125 kHz bandwidth — a 92 %
duty cycle at 10 Hz, which is not a margin, it is a coincidence. **Bandwidth is now
250 kHz**, halving airtime to 46.2 ms:

| SF7, CR4/5, 47 B | airtime | duty at 5 Hz | duty at 10 Hz | sensitivity |
|---|---|---|---|---|
| BW 125 kHz | 92.4 ms | 46 % | 92 % | −123 dBm |
| **BW 250 kHz** | **46.2 ms** | **23 %** | **46 %** | **−120 dBm** |
| BW 500 kHz | 23.1 ms | 12 % | 23 % | −117 dBm |

The 3 dB costs nothing here: at 2 km and +20 dBm the received signal is about −71 dBm,
roughly **49 dB of margin**. Airtime is the scarce resource in this link, not sensitivity.

All five radio parameters now live in `ariadna_link.h` and are read by both ends. They were
previously duplicated in two files under a "must match" comment — which is a comment, not a
guarantee.

**2. The transmitter no longer blocks, and never queues.** `endPacket(true)` is
asynchronous, and a slot whose predecessor is still on air is **skipped, not queued**.
Queuing is how a downlink degrades from "occasionally late" to "permanently a second
behind": each late packet delays the next and the backlog only grows. The Master counts
`sent` and `skipped` and prints both — if `skipped` climbs, `ARIADNA_TLM_HZ` is too high
for the airtime.

**3. The ground station stopped being blind half the time.** This was a real bug. A full
SSD1306 refresh is 1024 bytes over I²C, and the Arduino default clock is 100 kHz:

```
1024 bytes × 9 bits / 100 kHz = 92.2 ms per redraw
1024 bytes × 9 bits / 400 kHz = 23.0 ms per redraw
```

The receiver was redrawing every 100 ms at 100 kHz — blocked for 92 % of the interval
between packets. The SX1278 holds one packet in its FIFO so a single overlap survives, but
two in a row does not. Fixed by running I²C at 400 kHz, dropping the redraw to 5 Hz, and
**putting the display on core 1 with the radio alone on core 0.** The OLED is on I²C and the
radio on SPI1, so the split is clean.

> ⚠️ **Duty cycle regulation.** 433.05–434.79 MHz is an ISM band, and in the EU it is
> generally limited to a **10 % duty cycle at 10 mW ERP** under ETSI EN 300 220. 10 Hz at
> +20 dBm is well outside that. `make link` prints the figure and warns. Check your
> competition rules and national regulator; drop `ARIADNA_TLM_HZ` to 5 (23 %) or 2 (9 %),
> or reduce TX power, to comply. Nothing in the firmware enforces this — it is your call —
> but do not discover it at the pad.

### Coordinate conventions

Pad-local ENU: **+x East, +y North, +z Up**. `vz` is positive up, so it is negative for the
whole descent — the same sign the guidance module uses.

Attitude travels as a **quaternion** and as the **thrust-axis unit vector**, never as Euler
angles in the control path. `u` is the direction the landing motor accelerates the vehicle. Standing on the pad, `u = (0,0,-1)`; in a
perfect nose-first descent, `u = (0,0,+1)`. Defining it this way makes the guidance tilt
angle simply the angle between `u` and vertical, with no sign gymnastics and no singularity
where an Euler yaw would have one.

Angular rates are **the gyro's own measurements, rotated into the pad frame** — rotated,
never differentiated. In the guidance simulation, differencing a fused attitude instead
drops the landing rate from 72.6 % to 6.9 %.

---

## Annunciators — reading the vehicle without a laptop

On the pad there is no laptop. There is a vehicle on a rail, a crew behind a line, and
a few seconds in which somebody has to decide whether to arm it. Every board therefore
drives an indicator, and **each one answers a different question**:

| board | LED | answers |
|---|---|---|
| Master | SK6812 on GP2, buzzer on GP3 | *what is the mission doing?* |
| Position | XIAO on-board WS2812 | *does the vehicle know where it is?* |
| Navigation | XIAO on-board WS2812 | *is it ready to fire a pyro?* |
| Anchor | XIAO on-board WS2812 | *is this anchor ranging?* |

They are driven by one shared, non-blocking pattern engine,
[`common/ariadna_rgb.h`](common/ariadna_rgb.h), so that a colour means the same thing
everywhere in the system.

### Master — the mission state

Solid blue BOOT · slow green IDLE · **fast red ARMED** · solid amber ASCENT · solid
yellow APOGEE · solid magenta DESCENT · **solid white GUIDANCE** · solid green LANDED ·
slow green + beacon RECOVERY · fast amber ABORT · slow blue TEST · solid red ERROR.

The buzzer chirps once a second when ARMED and runs a 1 Hz beacon in RECOVERY. Nothing
else: a buzzer during flight is battery drain, and the beacon is the thing that finds a
vehicle in long grass.

### Position — the navigation solution

| indication | meaning |
|---|---|
| solid blue | booting |
| red, 1 flash | self test failed, or the IMU is not talking |
| red, 2 flashes | no barometer — the vertical channel has no absolute reference |
| slow amber | alive, but no pad reference: `px`/`py`/`pz` mean nothing yet |
| slow magenta | pad reference taken, no GPS and no UWB — dead reckoning only |
| slow cyan | navigating on GPS |
| **pulsing green** | navigating on GPS, UWB constellation alive |
| **solid green** | UWB fixes are being *accepted by the filter* |

The distinction between the last two is the one worth understanding from behind the
safety line. Pulsing green says the anchors are powered. Solid green says they are
powered, surveyed correctly, and agreeing with the rest of the navigation solution.
**Solid green is the one to wait for before arming.**

### Navigation — the pyro state

| indication | meaning |
|---|---|
| solid red | self test failed. Do not arm. |
| **red, 2 flashes** | armed, but channel 1 has **no continuity** |
| slow blue | alive, waiting for a navigation state |
| slow amber | navigation state arriving, not armed |
| slow green | armed and receiving, continuity on channel 1 |
| fast green | the landing burn is running |
| fast magenta | the navigation state has gone stale; the TVC is parked |

### Anchor

Solid red = the DWM1000 did not come up. Slow blue = up, but the tag has never called.
Pulsing green = ranging (what all four should look like during a countdown). Fast amber
= it *was* ranging and the tag has been quiet for five seconds.

### Why this is affordable inside a 200 Hz loop

Driving a WS2812 means bit-banging 24 bits at 800 kHz with interrupts off — about 30 µs
during which nothing else on that core happens. Two properties keep that harmless:
`ariadna_rgb_service()` is a state machine that never delays or blocks, and it pushes to
the LED **only when the computed pixel value differs from the one already on the wire**.
A solid colour therefore costs one transfer and then nothing at all; the fastest pattern
in the vehicle costs two transfers a second. Every board calls it at the very end of its
tick, after the work that matters.

---

## What is verified, and what is not

This matters more than anything else in this README.

| | how |
|---|---|
| **guidance algorithm** | 124 numerical checks against the Monte-Carlo simulation, including the properties that make the two-axis form correct: exact reduction to the planar law, isotropy in azimuth, an orthonormal gimbal basis, the dynamic-inversion round trip, a pure roll rate moving neither servo, and the gimbal axes tracking the airframe through a full 360° roll. |
| **link codec** | 111 checks: round trip, CRC, resync after garbage, 2000-frame randomly corrupted stream, transmit ring, link budget. |
| **navigation filter** | 47 checks: `make est`, including the barometer trim observer. `ariadna_kf.h` compiles unchanged into the test and into the sketch, so the arithmetic checked is the arithmetic that flies. |
| **UWB multilateration** | 108 checks: `make uwb`. Every degraded case — four anchors down to one and none, NaN and negative ranges, coincident and collinear anchors, a 10 m reflection, a 600-trial Monte Carlo the quoted σ must match, the whole descent profile from 50 m/s down, and the vertical envelope to the ground. `ariadna_uwb.h` likewise compiles unchanged into both. |
| **header copies** | `make sync` proves every sketch's copy of the guidance module and of `common/` is byte-identical to the original. |
| **Python decoder** | `make pyfmt` pins the visualiser's struct format against the C header. |
| **Navigation sketch** | compiles clean under `-Wall -Wextra -Wdouble-promotion`. |
| **DMA channel setup** | ⚠️ **not testable off-target.** The ring it drives is tested; the channel configuration is not. |
| **estimator, end to end** | ⚠️ **measured, not in `make check`.** The closed-loop descent numbers above were produced by compiling the `Estimator` class on the host, which needs the sketch sliced by hand because that class lives in the `.ino`. The pieces it is built from — the filter, the trim observer, the multilateration — are all in `make check`. Moving the whole estimator into a header is the obvious next refactor. |
| **estimator wiring** | ⚠️ **partly verified.** The filter arithmetic is covered by `make est`; how the sketch feeds it — sensor axes, GPS parsing, the pad reference — is not. Boot self-test checks convergence, the barometric conversion, the innovation gate and quaternion/matrix agreement. |
| **UWB anchor survey** | ⚠️ **cannot be verified in software at all.** The boot self test catches duplicated addresses, anchors closer than a metre and a geometry that cannot solve; it cannot catch an anchor that is simply not where the table says. Measure it, and watch the post-fit residual. |
| **UWB driver** | ⚠️ **not host-compiled with the radio present** — the ranging library is absent from the host build. The geometry is fully tested; the SPI and IRQ handling is not. |
| **body→pad rotation** | ⚠️ **not verified.** Three conventions have to agree and only one is ours. |
| **Master, Position, ground station sketches** | not host-compiled — they pull in LoRa, SSD1306 and TriSense. All five sketches build clean with `arduino-cli` (arduino-pico 6.2.0, `--warnings all`); nothing in `make check` does that. |

The three ⚠️ rows share a failure mode: they do not produce a plausible-but-wrong number,
they mirror an axis. A mirrored axis turns the attitude loop into positive feedback, which
diverges in well under a second. **Bench test 3 below exists for exactly this and is not
optional.**

### There is no longer a 3D→planar bridge

`guidance.cpp` now solves the full three-dimensional problem with a two-axis gimbal, so
section 6 of `Navigation_MCU.ino` is almost a **copy** rather than a transform: the link's `u`
vector is `dg_nav_t.b` and the link's `w` is `dg_nav_t.w`, identically defined at both ends.

The one thing it derives is the airframe's **roll orientation**. The TVC servos are bolted to
the vehicle, so as it spins the direction each one pushes spins with it, and the guidance needs
a body-fixed reference axis to track them:

```
g = R(q) * BODY_SERVO1_AXIS
```

— the same construction the Position MCU already uses for `u`, from the quaternion the link
already carries. Deriving it on the Navigation MCU rather than adding it to the wire keeps
`ariadna_nav_state_t`, the link tests and the Python decoder untouched, and costs ~30 flops.
`BODY_SERVO1_AXIS` **must** match the axis `TVC_SIGN_X` drives at the Master; if the two
disagree by 90° the channels are simply swapped, and every static bench test still passes.

An earlier version rotated the 3-D state into a vertical guidance plane aligned with the
horizontal velocity, ran a planar law inside it, and split the single servo angle back onto
two axes. That worked, but it had to **freeze** the plane azimuth below 0.5 m/s of horizontal
speed (`atan2` of noise spins it through 360° per tick) and freeze it again at ignition
(re-aiming mid-burn rotates the attitude reference underneath a closed loop that assumes it
is fixed). The two-axis law has neither restriction: it controls both channels directly, at
any azimuth, for the whole burn.

Removing it also removed the riskiest code in the file — a rotation four sign conventions
deep, where a mistake produced positive feedback rather than a wrong number. **The two servo
signs at the Master are still hardware conventions and still need bench test 3.**

---

## Running the tests

```
cd ../test
make          # 124 guidance checks against the simulation
make est      # 47 navigation filter and barometer-trim checks
make uwb      # 108 UWB checks: degraded constellations and a moving vehicle
make link     # 111 link codec checks, the transmit ring, and the link budget
make sync     # every sketch's copy of guidance and common/ matches the original
make pyfmt    # ground_visualizer.py matches the C packet layout
make syntax   # host-compile the sketches that can be host-compiled
make check    # all of it
```

`make sync-fix` re-copies the guidance module and the `common/` headers into every sketch
folder after a change to an original. Arduino cannot reach anything outside the sketch
folder, which is why the copies exist — and why they are *checked* rather than trusted.

---

## Building

Arduino IDE 2, board package **arduino-pico** (earlephilhower).

| sketch | board | libraries |
|---|---|---|
| `Master_MCU/Master_MCU.ino` | Raspberry Pi Pico 2 W | LoRa (sandeepmistry), Adafruit_NeoPixel |
| `Position_MCU/Position_MCU.ino` | Seeed XIAO RP2350 | [Voltino TriSense](https://github.com/Tomasraketak/Voltino_TriSense_Library), [arduino-dw1000](https://github.com/thotro/arduino-dw1000)\*, Adafruit_NeoPixel |
| `Navigation_MCU/Navigation_MCU.ino` | Seeed XIAO RP2350 | Adafruit_NeoPixel |
| `Ground_Station/Ground_Station.ino` | Raspberry Pi Pico 2 | LoRa, Adafruit_GFX, Adafruit_SSD1306 |
| `Uwb_Anchor/Uwb_Anchor.ino` | Seeed XIAO RP2350 ×4 | arduino-dw1000, Adafruit_NeoPixel |

\* **Optional.** Without arduino-dw1000, set `ARIADNA_UWB_ENABLED` to 0 at the top of
`Position_MCU.ino`: the entire UWB path compiles out and the board flies on GPS, barometer
and IMU exactly as it did before. The same switch rules the radio out deliberately — for
instance while chasing an SPI fault. It is a plain switch rather than a `__has_include`
probe because the Arduino builder only adds a library's include path once it sees a real
`#include`, so the probe always read "absent" and the radio was compiled out silently.

The registry release of arduino-dw1000 (0.9.0) does not build on arduino-pico as shipped:
`DW1000RangingClass::detectMessageType()` in `DW1000Ranging.cpp` can fall off its end, which
this core compiles with `-Werror=return-type`. Add `return -1;` as its last line.

Adafruit_NeoPixel is optional in the same way: without it the annunciator calls compile
to nothing, which is what lets `make syntax` build the sketches against a stub Arduino
header with no libraries at all.

Before the first flight firmware goes on, flash the bench tests in
[`Bench_Tests/`](Bench_Tests/README.md): one sketch per board that checks every sensor,
servo, memory, radio and inter-board link and prints the verdicts on USB serial.

Flash the four anchors from one sketch by changing `ANCHOR_ID` to 1, 2, 3, 4 in turn.
**Write the number on the box** — an anchor whose ID does not match where it physically
stands is the one failure in this system that produces a confident, wrong position
rather than no position.

Each sketch folder is self-contained: it carries its own copy of the `common/` headers it
uses (and the Master a copy of `guidance.h`), because the Arduino builder compiles a sketch
from a copy of its folder and a `"../common/…"` include does not resolve there. `common/`
holds the originals. Edit those, then `cd ../test && make sync-fix`; `make sync` (part of
`make check`) fails if any copy has drifted. The `.ino` in each folder carries the folder's
name, which Arduino also requires.

`common/` holds five headers, and the split is deliberate:

| header | is | tested by |
|---|---|---|
| `ariadna_link.h` | the wire format | `make link`, `make pyfmt` |
| `ariadna_uart_dma.h` | the only part with a hardware dependency | `make link` (the ring) |
| `ariadna_kf.h` | the navigation filter's arithmetic and the barometer trim | `make est` |
| `ariadna_uwb.h` | the UWB multilateration | `make uwb` |
| `ariadna_rgb.h` | the annunciator pattern engine | `make syntax` |

The arithmetic is kept out of `Position_MCU.ino` for the same reason `guidance.cpp` is
kept out of the Navigation MCU's sketch. Anything inside a `.ino` can only be tested by
flashing a board, and the Position MCU sketch cannot be host-compiled at all because it
pulls in TriSense. The filter and the multilateration can be, and are — and the headers
that are checked on a workstation are the *same files* that compile into the sketch, not
copies of them.

Visualiser:

```
pip install pyserial PyQt6 PyQt6-WebEngine pyqtgraph haversine
python Ground_Visualizer/ground_visualizer.py
```

---

## Pinout

**The complete pinout of the whole avionics — all five boards, the PCA9685 channels, the
inter-board wiring, the power rails and the interlocks — is in
[`PINOUT.md`](PINOUT.md).** It is derived from the spreadsheet's *"Přiřazení pinů 2"*
sheet and it names, for every pin, the constant in the sketch that actually drives it.

The sketches use the XIAO silkscreen aliases (`D0`…`D12`) that the board variant defines
rather than raw GPIO numbers, so they stay correct regardless of how Seeed numbers the die.

### Three notes on the pinout

**The spreadsheet labels the XIAO UARTs "UART1".** On the arduino-pico core `Serial1` is
UART0 and `Serial2` is UART1, and which instance reaches D6/D7 depends on the variant's GPIO
mapping. Both sketches select the port through a single `#define MASTER_PORT` — if a board
is silent on the bench, change that one line before touching anything else.

**The GPS runs on a PIO state machine**, not a hardware UART. D11 and D12 are on the
underside of the XIAO and are not guaranteed to land on one. The RP2350 will put a UART on
any pin, which is one of the places its architecture saves a design compromise rather than
just being fast.

**D0 and D1 on the Position MCU are not a UART either.** The spreadsheet's second column
names the pin's *function* and its third names what is *physically on it*; where the two
disagree, the third wins. D0 and D1 carry the DWM1000's interrupt and reset lines, and the
reset is driven open-drain — pulled LOW to reset, then released to a floating input, because
driving it high can damage the module.

---

## The servo path, and why it is the way it is

The pinout puts every servo on the Master's PCA9685, but the 200 Hz loop that commands them
runs on the Navigation MCU. So a nozzle command crosses a UART and an I²C bus:

```
guidance solve → UART 460800, 42 B   ~0.9 ms
               → Master core-0 tick   ≤ 5 ms
               → PCA9685 I²C write   ~0.3 ms
```

Budget **3–6 ms** end to end. At the guidance loop's 9 rad/s bandwidth that is a few degrees
of phase — acceptable, but only because the Master's servo path does nothing else. Actuator
packets are applied to the PCA9685 *inside* the UART drain loop rather than at the end of the
tick. Do not add work there.

The PCA9685 runs at **339 Hz**, not the usual 50 Hz. A 50 Hz refresh adds up to 20 ms of dead
time on its own, more than the rest of the chain put together.

> ⚠️ **Digital servos only.** The PCA9685's PWM frequency is global across all sixteen
> channels. Every servo on that chip sees 339 Hz, and most analogue servos will buzz or
> overheat.

| channel | use |
|---|---|
| 0 / 1 | TVC X / Y |
| 2 | thrust clamp |
| 3–6 | aerodynamic fins *(future)* |
| 7 | spare |

---

## Pyros

Three channels on the Navigation MCU, with continuity sense on each.

| channel | use |
|---|---|
| 1 | landing motor igniter |
| 2 | landing motor igniter, **backup** |
| 3 | reserve — wired and monitored, never fired automatically |

Channel 1 fires on the guidance module's ignition decision. Then the vehicle either
accelerates or it does not: if axial acceleration has not risen above 2 g **250 ms** later,
channel 2 fires. The window is deliberately short. The grain burns for 1.75 s and the
ignition altitude has no margin for a late light — waiting long enough to be certain is the
same as not firing at all.

Four independent gates must all hold before any channel can fire: the Master must be
present, its state must be fresh, it must say ARMED, and it must not say TEST or ABORT. The
arming check is repeated at the output stage, so withdrawing arming **cuts current** rather
than merely preventing the next firing.

---

## Mission state machine

`BOOT → IDLE → ARMED → ASCENT → APOGEE → DESCENT → GUIDANCE → LANDED → RECOVERY`

Arming needs the RBF tag out, a healthy navigation solution, and the vehicle actually
sitting still on the pad. The pad pressure reference is captured **at arming**, not at boot
— the vehicle may have sat on the pad for an hour while the weather moved.

Launch is detected by the breakwire parting, *or* by 3 g of acceleration, *or* by the vehicle
simply being above 15 m. A breakwire that fails to part is a known failure and it must not
strand the state machine on the pad while the rocket is in the air.

At 85 m the state machine reports handover to the Navigation MCU. It **reports** it — the
Navigation MCU arms itself on the same altitude from the same header constant
(`DG_H_START`, included from `guidance.h` so the two cannot disagree). Nothing on the Master
can hurry the burn.

---

## Flash logging

88-byte records at 50 Hz — 4.4 kB/s, about an hour on a 16 MB W25Q128.

A sector erase takes up to 400 ms and nothing can be programmed while it runs, so the driver
**erases ahead** of the write pointer and buffers arrivals in an 8 kB RAM ring. Core 0 never
waits: if the ring is full it drops the record and counts it, because a dropped log line is
better than a missed servo command.

At boot the append point is found by binary search — the log is append-only, so "erased" is
monotone in sector index and 12 reads find the boundary instead of 4096.

---

## Telemetry

47 bytes at 4 Hz over LoRa, SF7 / 125 kHz / CR4-5, which is 92 ms on air and a 37 % duty
cycle. `make link` prints the figure. Raising the rate or the packet size eats that margin
directly, and the 433 MHz ISM band has duty-cycle limits in the EU that this is already close
to. If you need a faster downlink, widen the bandwidth to 250 kHz — that halves airtime for
about 3 dB of link budget — rather than shortening the packet.

The V2 packet is **not compatible with the V1 ground station**. It carries mission state,
guidance phase, TVC deflection, thrust clamp, thrust estimate, pyro continuity and per-board
health alongside everything V1 had, and GPS resolution goes from 1e-6 to 1e-7 degrees.

The most recent addition is a single **UWB status byte**, and it was argued for on exactly
the airtime grounds above: one byte against being able to see a dead anchor from the pad. It
packs the anchor count, the solution mode, whether the filter is *accepting* UWB fixes, and
whether the radio is alive. Reading `0x00` during a countdown means the anchors are not
powered; reading `0x84` — alive, four anchors, no accepted fix — means they are powered and
the filter is rejecting them, which is a survey error and is fixable in the ten minutes
before launch. `ARIADNA_TLM_VERSION` is 3; a V2 packet is refused by the visualiser rather
than misread.

The USB link from the receiver to the laptop is **framed** — sync word, length, CRC — where
V1 wrote a bare struct. V1 had no way to resynchronise: one dropped byte and every packet
after it decoded as garbage until the app was restarted. The framed version self-heals on
the next packet, and the CRC means the visualiser can tell a corrupted frame from a real
reading of the same value.

---

## Before you fly this

This software is **unflown**, and most of it is untested by anything stronger than its own
boot self-test. The pin map, servo travel, calibration constants, thresholds and polarities
describe an *intended* vehicle. Check every one against yours.

Suggested order:

1. **Bench each board alone.** Confirm POST passes on the USB console. Navigation MCU with
   igniters disconnected and the Master unpowered — pyros are inhibited when the Master's
   state is stale, so this also proves that interlock.

2. **Bench the links.** Power all three. Confirm `rx_ok` climbs and `rx_bad` does not on
   every console. Confirm the Navigation MCU's `loop_us` and `overruns` stay near zero.

2b. **Bench the UWB constellation, before you need it.** Power the four anchors and confirm
   each one goes to pulsing green. Then put the vehicle at a *measured* point — a tape
   measure and a known corner of the anchor square is enough — and check that `px`/`py` in
   `POS_STATUS` agree with where it actually is, and that the post-fit residual settles below
   about 20 cm. A residual that will not come down, or a position that is consistently offset,
   is a survey error in `s_uwb_anchors[]`, not a radio problem. Then **unplug one anchor** and
   confirm the solution stays sane and the anchor count in the downlink drops to three. Then a
   second. The vehicle must still be navigating.

3. **⚠️ Check the TVC signs.** Loop running, dummy thrust estimate, no motor. Deflect the
   airframe by hand along +x and confirm the nozzle swings to **oppose** the motion. Then
   along +y. Then both. If it follows instead, `TVC_SIGN_X` / `TVC_SIGN_Y` in
   `Master_MCU.ino` are wrong, and the vehicle will diverge in under a second. Test **both**
   axes — testing one and assuming the other is how a sign error reaches the pad. **This test
   catches the single most dangerous class of bug in the whole repository.**

3b. **Check for roll.** Rotate the airframe about its own long axis and confirm it stays where
   you put it. The guidance assumes roll is zero and cannot correct it; a rolling body also
   turns the two gimbal axes underneath the controller, so the channels smear together.

4. **Check the attitude conventions.** Point the nose North, then East; tip it 10° each way.
   Confirm `u` and the reported roll/pitch/yaw move the way `ariadna_link.h` says. This is
   where a compass-heading-versus-azimuth mistake shows up.

5. **Calibrate continuity.** Measure the ADC counts on each pyro channel with a known-good
   igniter and with an open channel. Put the midpoint in `CONT_THRESHOLD` and check
   `CONT_ACTIVE_HIGH`. Getting the polarity backwards means the board reports a good igniter
   on an empty channel.

6. **Static fire** with the clamp servo connected and the TVC locked, to calibrate
   `CLAMP_US_CLOSED` / `CLAMP_US_OPEN` against measured thrust.

7. **Range test the radio** at the distance you actually need, with the antennas you will
   actually fly, before trusting the link quality indicator.

Only then fly it, over a range, with the usual model-rocketry safety practice for the motors
involved.
