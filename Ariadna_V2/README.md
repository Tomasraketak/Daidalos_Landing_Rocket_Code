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
| **Position** | XIAO RP2350 | sensor fusion, the navigation solution |
| **Navigation** | XIAO RP2350 | guidance, ignition timing, TVC, pyros |
| **Ground station** | Raspberry Pi Pico 2 | LoRa receive, OLED, USB uplink to the laptop |

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
```

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

**The inter-MCU links run at 460800, not 115200.** They are short PCB traces, so the higher
rate costs nothing, and it keeps each stream under a quarter of the link:

```
Position→Master   nav state @240 Hz + status : 33.1 %
Master→Navigation nav state @240 Hz          : 30.2 %
Navigation→Master actuator @200 Hz + status  : 20.6 %
```

At 115200 the same traffic sits near 80 %, and every field added after that comes out of
control latency. Two `#define`s at the top of `ariadna_link.h` change it back if you want
115200; `make link` prints the resulting budget.

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

To pay for 240 Hz the packet was cut from 84 to 52 bytes: attitude in degrees, latitude,
longitude, GPS altitude and temperature moved to `POS_STATUS` at 20 Hz. That halves the
cost of the highest-rate thing on the link — and it crosses the link *twice*, Position to
Master and Master to Navigation. Nothing was lost: every field that moved is telemetry,
GPS updates at 10 Hz at best, and a navball redrawn at 20 Hz looks identical to one
redrawn at 240 Hz. **Before adding a field to `nav_state`, ask whether a control law reads
it.**

#### Predict at 240 Hz, correct at 50 Hz

The estimator's *prediction* step runs every tick. The barometer *correction* runs only on
a genuinely new sample — about 50 Hz, which is what the BMP580 delivers at the OSR ×16 this
build uses for its altitude noise floor.

That distinction matters. Applying the correction on every predict step would feed the same
measurement in roughly five times, multiplying the effective gain by five and moving the
filter's crossover well above the bandwidth it was tuned for. It would still converge — it
would just be a different filter than `BARO_W_NOMINAL` claims. The correction also
integrates over the interval since the *last correction*, not over one prediction step.

Sampling the barometer faster would not help: the vertical filter believes it below about
0.3 Hz and the accelerometer above, so 50 Hz is already more than a hundred times
oversampled for the job. A quieter sample at 50 Hz beats a noisier one at 240 Hz here.

### DMA, and where it is deliberately absent

The processor is not short of cycles. CRC and frame parsing together cost about **1 % of a
core** at full traffic — measured, not assumed, and printed by `make link`.

What *was* expensive is different: the UART's hardware transmit FIFO is 32 bytes, and a byte
takes 21.7 µs at 460800 baud. A frame longer than the FIFO cannot be handed over in one go.

| | bytes past the FIFO | per frame | duty |
|---|---|---|---|
| Master forwards `nav_state` @240 Hz | 26 | 564 µs | **13.5 %** of a core |
| Position sends `nav_state` @240 Hz | 26 | 564 µs | **13.5 %** of a core |
| Navigation sends `actuator` @200 Hz | 10 | 217 µs | **4.3 %** of a core |

A blocking write spends that time spinning. Worse than the wasted cycles, on the Master
those 1.26 ms land *inside the forwarding path* — forwarding the frame before parsing it
saves microseconds, and then blocking on the FIFO gives back a millisecond.

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

**1. Airtime.** A 46-byte payload at SF7 is 92.4 ms on air at 125 kHz bandwidth — a 92 %
duty cycle at 10 Hz, which is not a margin, it is a coincidence. **Bandwidth is now
250 kHz**, halving airtime to 46.2 ms:

| SF7, CR4/5, 46 B | airtime | duty at 5 Hz | duty at 10 Hz | sensitivity |
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

Attitude travels as the **thrust-axis unit vector**, not Euler angles: `u` is the direction
the landing motor accelerates the vehicle. Standing on the pad, `u = (0,0,-1)`; in a
perfect nose-first descent, `u = (0,0,+1)`. Defining it this way makes the guidance tilt
angle simply the angle between `u` and vertical, with no sign gymnastics and no singularity
where an Euler yaw would have one.

Angular rates are **the gyro's own measurements, rotated into the pad frame** — rotated,
never differentiated. In the guidance simulation, differencing a fused attitude instead
drops the landing rate from 72.6 % to 6.9 %.

---

## What is verified, and what is not

This matters more than anything else in this README.

| | how |
|---|---|
| **guidance algorithm** | 108 numerical checks against the Monte-Carlo simulation. Unchanged from V1. |
| **link codec** | 107 checks: round trip, CRC, resync after garbage, 2000-frame randomly corrupted stream, transmit ring. |
| **guidance copy** | `make sync` proves the Navigation MCU's copy is byte-identical to the verified original. |
| **Python decoder** | `make pyfmt` pins the visualiser's struct format against the C header. |
| **Navigation sketch** | compiles clean under `-Wall -Wextra -Wdouble-promotion`. |
| **DMA channel setup** | ⚠️ **not testable off-target.** The ring it drives is tested; the channel configuration is not. |
| **3D→planar bridge** | ⚠️ **new code, no golden vectors.** Boot self-test checks the arithmetic. Only a bench checks the conventions. |
| **state estimator** | ⚠️ **not verified.** Boot self-test checks filter convergence and the barometric conversion. |
| **body→pad rotation** | ⚠️ **not verified.** Three conventions have to agree and only one is ours. |
| **Master, Position, ground station sketches** | not host-compiled — they pull in LoRa, SSD1306 and TriSense. Build them in Arduino IDE. |

The three ⚠️ rows share a failure mode: they do not produce a plausible-but-wrong number,
they mirror an axis. A mirrored axis turns the attitude loop into positive feedback, which
diverges in well under a second. **Bench test 3 below exists for exactly this and is not
optional.**

### The 3D→planar bridge

`guidance.cpp` solves a planar problem — one altitude, one horizontal axis, one tilt, one
servo. Daidalos flies in three dimensions with a two-axis gimbal. Section 6 of
`navigation_mcu.ino` bridges them by choosing a vertical **guidance plane** aligned with the
horizontal velocity, rotating the state into it, running the verified law, and decomposing
the single servo angle back onto two axes.

The plane is frozen below 0.5 m/s of horizontal speed (otherwise `atan2` of noise spins it
through 360° per tick) and frozen again once the burn starts (re-aiming it mid-burn would
rotate the attitude reference underneath a closed loop that assumes it is fixed).

---

## Running the tests

```
cd ../test
make          # 108 guidance checks against the simulation
make link     # 107 link codec checks, the transmit ring, and the link budget
make sync     # the Navigation MCU's guidance copy has not drifted
make pyfmt    # ground_visualizer.py matches the C packet layout
make syntax   # host-compile the sketches that can be host-compiled
make check    # all of it
```

`make sync-fix` re-copies `guidance.h` / `guidance.cpp` into `Navigation_MCU/` after a change
to the original. Arduino cannot compile a `.cpp` outside the sketch folder, which is why the
copy exists — and why it is *checked* rather than trusted.

---

## Building

Arduino IDE 2, board package **arduino-pico** (earlephilhower).

| sketch | board | libraries |
|---|---|---|
| `Master_MCU/master_mcu.ino` | Raspberry Pi Pico 2 W | LoRa (sandeepmistry), Adafruit_NeoPixel |
| `Position_MCU/position_mcu.ino` | Seeed XIAO RP2350 | [Voltino TriSense](https://github.com/Tomasraketak/Voltino_TriSense_Library) |
| `Navigation_MCU/navigation_mcu.ino` | Seeed XIAO RP2350 | none |
| `Ground_Station/ground_station.ino` | Raspberry Pi Pico 2 | LoRa, Adafruit_GFX, Adafruit_SSD1306 |

The sketches reach the shared header by relative path (`#include "../common/ariadna_link.h"`).
Arduino IDE 2 resolves quoted includes relative to the including file, so one copy serves all
four. Keep the directory layout as it is.

Visualiser:

```
pip install pyserial PyQt6 PyQt6-WebEngine pyqtgraph haversine
python Ground_Visualizer/ground_visualizer.py
```

---

## Pinout

From the spreadsheet, unchanged. `D0..D12` are the XIAO silkscreen aliases the board variant
defines — the sketches use those names rather than raw GPIO numbers, so they stay correct
regardless of how Seeed numbers the die.

**Master (Pico 2 W)**

| pin | function |
|---|---|
| GP0 / GP1 | UART0 ↔ Position MCU |
| GP2 | SK6812 RGB |
| GP3 | buzzer |
| GP4 / GP5 | UART1 ↔ Navigation MCU |
| GP6 / GP7 | reset lines to Position / Navigation |
| GP8 / GP9 | I²C0 → PCA9685 |
| GP10 | breakwire, launch detect |
| GP12–GP15 | SPI1 → W25Q128 flash |
| GP16–GP21 | SPI0 + DIO0 + RST → LoRa Ra-02 |
| GP22 | spare digital out ("S16") |
| GP26 / GP27 / GP28 | current sense / battery / RBF |

**Position (XIAO RP2350)**

| pin | function |
|---|---|
| D0 / D1 / D2 | UWB INT / RST / CS *(future)* |
| D3 | TriSense CS |
| D4 / D5 | I²C — BMP580, AK09918C, TF Luna *(future)* |
| D6 / D7 | UART ↔ Master |
| D8 / D9 / D10 | SPI — ICM42688P, UWB |
| D11 / D12 | LK76 GPS, on a **PIO** UART |

**Navigation (XIAO RP2350)**

| pin | function |
|---|---|
| D0 / D1 / D2 | pyro continuity 1 / 2 / 3 |
| D3 | pyro 1 — landing motor igniter |
| D4 / D5 | I²C, broken out |
| D6 / D7 | UART ↔ Master |
| D8 | pyro 2 — landing motor igniter, **backup** |
| D9 | pyro 3 — reserve, never fired automatically |
| D10 | spare |
| USB-C | pad console |

### Two notes on the pinout

**The spreadsheet labels the XIAO UARTs "UART1".** On the arduino-pico core `Serial1` is
UART0 and `Serial2` is UART1, and which instance reaches D6/D7 depends on the variant's GPIO
mapping. Both sketches select the port through a single `#define MASTER_PORT` — if a board
is silent on the bench, change that one line before touching anything else.

**The GPS runs on a PIO state machine**, not a hardware UART. D11 and D12 are on the
underside of the XIAO and are not guaranteed to land on one. The RP2350 will put a UART on
any pin, which is one of the places its architecture saves a design compromise rather than
just being fast.

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

46 bytes at 4 Hz over LoRa, SF7 / 125 kHz / CR4-5, which is 92 ms on air and a 37 % duty
cycle. `make link` prints the figure. Raising the rate or the packet size eats that margin
directly, and the 433 MHz ISM band has duty-cycle limits in the EU that this is already close
to. If you need a faster downlink, widen the bandwidth to 250 kHz — that halves airtime for
about 3 dB of link budget — rather than shortening the packet.

The V2 packet is **not compatible with the V1 ground station**. It carries mission state,
guidance phase, TVC deflection, thrust clamp, thrust estimate, pyro continuity and per-board
health alongside everything V1 had, and GPS resolution goes from 1e-6 to 1e-7 degrees.

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

3. **⚠️ Check the TVC signs.** Loop running, dummy thrust estimate, no motor. Deflect the
   airframe by hand along +x and confirm the nozzle swings to **oppose** the motion. Then
   along +y. Then both. If it follows instead, `TVC_SIGN_X` / `TVC_SIGN_Y` in
   `master_mcu.ino` are wrong, or the guidance-plane azimuth is mirrored, and the vehicle
   will diverge in under a second. **This test catches the single most dangerous class of
   bug in the whole repository.**

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
