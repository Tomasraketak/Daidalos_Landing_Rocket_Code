# Static Fire Link — Ariadna fires the motor on the stand and records the burn

A bench example: Ariadna V2 and the [Static Fire Stand](https://github.com/Tomasraketak/Static_Fire_Stand)
(the load-cell test stand, Raspberry Pi Pico 2 W) joined by I2C.

1. Both boards say they are **READY** to each other.
2. The operator starts the countdown on the **stand** (physical button or the WiFi page), as always.
3. Ariadna watches the countdown, arms **pyro 1**, and when the stand reaches T0 and says *ignite*,
   fires it.
4. The **clamp** ("klapka"), fully open until then, runs a profile as soon as the thrust passes 100 N:
   **50 % open → 300 ms → minimum → 300 ms → 75 % → 300 ms → fully open.**
5. The stand records the thrust the whole time (its own power-fail safe log). Ariadna records battery
   voltage, current, the clamp pulse and the clamp closure.
6. When the burn is over, Ariadna **copies the stand's whole record over I2C into its own W25Q128
   flash and merges** it with its own data: one row per thrust sample, with voltage, current, servo
   position, clamp closure and time beside it.

| part | runs on | file |
|---|---|---|
| this example | **Master** — Raspberry Pi Pico 2 W | `Static_Fire_Link.ino` |
| pyro 1 (igniter) | **Navigation** — Seeed XIAO RP2350 | [`../Pyro_Companion/Pyro_Companion.ino`](../Pyro_Companion/Pyro_Companion.ino), unchanged |
| load cell, countdown, the stand's record | the stand | `Static_Fire_Stand`, firmware **2.1+**, `link ariadna` — see its [docs/ARIADNA_LINK.md](https://github.com/Tomasraketak/Static_Fire_Stand/blob/main/docs/ARIADNA_LINK.md) |

Needs **Adafruit_NeoPixel**; the PCA9685 and the W25Q128 are driven by their commands. Like every example
it **replaces the flight firmware** on the Master and Navigation boards and is flashed back afterwards.

## Wiring

| Stand | | Ariadna Master |
|---|---|---|
| GP4 (SDA) | ↔ | `I2C1` or `I2C2` uSup, SDA |
| GP5 (SCL) | ↔ | `I2C1` or `I2C2` uSup, SCL |
| GND | ↔ | uSup GND |

**Leave the 3V3 pins unconnected** — two supplies must not be tied. That uSup is the PCA9685 bus
(GP8/GP9, 10 kΩ pull-ups, 400 kHz): the stand is one more slave on it, `0x2C` beside the servo driver
at `0x40`. The pull-ups are Ariadna's; the stand has none. This is deliberate for a bench setup — the
flight umbilical stays on the Navigation `I2CN` and is not involved ([PINOUT.md](../../PINOUT.md) §7).

* **Igniter on Ariadna pyro 1** (Navigation MCU D3). Nothing on the stand's own ignition output.
* **Clamp servo on PCA9685 channel 7**, as in flight.
* **RBF key:** pulled on *both* boards to arm. The stand ignores a countdown with its key in; Ariadna
  will not be READY with its own key in.

## First time

1. **Flash** `Pyro_Companion` on the Navigation MCU and `Static_Fire_Link` on the Master (115200,
   line ending *Newline*).
2. **Set the numbers that are yours** at the top of `Static_Fire_Link.ino`, with the linkage unhooked:
   `CLAMP_US_OPEN` / `CLAMP_US_CLOSED` (the flight parameters `srv.clamp_open_us` / `srv.clamp_closed_us`)
   and `CH_CLAMP`. The defaults are the flight defaults, **not** your calibration.
3. **Continuity:** `CONT_THRESHOLD` in `Pyro_Companion.ino` is a property of *your* harness (see the
   Navigation bench test). Until it is right, Ariadna reports "no igniter on pyro 1" and will not be
   READY. `REQUIRE_CONT 0` is for a bench without an igniter.
4. **Stand:** `link ariadna`, `reboot` (or `python static_fire.py --link ariadna`).
5. On Ariadna's console `s` shows one line; `i` the same as JSON. When both are happy the LED is dim
   green and the stand's page says **Ariadna READY**.

| LED (Master) | meaning |
|---|---|
| blue blink | no link to the stand |
| dim green | linked, ready |
| yellow | linked, **not ready** (the console says why: `s`) |
| amber flashing + buzzer chirp | pyro 1 ARMED, counting down |
| red | burning |
| cyan | copying the record / writing flash |
| red/blue alternating | PCA9685 not answering |
| magenta flashing | fault (the pyro did not arm, or IGNITE came late); clears when the stand is idle |

## A run

Safety first — the same checklist as the stand's own README: people clear, motor restrained,
line of sight, **RBF keys out last, on both boards**.

1. Console shows `# local checks all fine`; the stand shows READY and **Ariadna READY**.
2. Start the countdown on the stand. Ariadna prints `# countdown seen`, `# pyro 1 ARMED`.
3. T0. `# T0: FIRE pyro 1 (300 ms), 3000 us after T0` — the number is how long after the stand's T0 the
   pulse went out (a few ms: Ariadna polls at 200 Hz around T0).
4. `# thrust over 100 N: clamp 50 % open`, then `# clamp step 1…`, `2`, `3`.
5. `# burn over`, then `# the stand has a finished burn: copying`, then
   `# run #3 stored in slot 2: 1040 rows, 38 stand pages, 2600 own samples, 700 ms (complete)`.
6. The stand's START button is available again.

If anything Ariadna needs goes away during the countdown (link, pyro arming, RBF, continuity), it says
**not ready** and **the stand stops the countdown**. If the stand goes away during the burn, the
clamp is driven fully open and stays there.

## The clamp profile

| constant | default | |
|---|---|---|
| `TRIGGER_N` | 100 | thrust that starts it (held for 10 ms) |
| `OPEN_PCT_1` | 50 | first step, % **open** |
| `OPEN_PCT_MIN` | 10 | "minimum": the flight software's own floor `DG_CLAMP_MIN` = 0.10, i.e. 90 % closed. `0` = shut as far as the linkage goes |
| `OPEN_PCT_3` | 75 | third step |
| `STEP_MS` | 300 | length of each of the three steps |
| after that | 100 | fully open until the end |

Percentages are how far the clamp is **open** (100 = the flight firmware's position at ignition); the
record's *closure* column is `100 − open`. No trigger within 8 s of T0 → the clamp stays open and the
sequence is over. **The servo position in the record is the commanded pulse**: a hobby servo gives no
feedback, so it is what Ariadna told it, not what it did.

## What is stored

On Ariadna's W25Q128, the **top 1 MB**, eight slots of 128 kB used round-robin (a slot is erased in the
background after each run). Sectors 0–1 (the flight parameter table) and the flight log growing from
sector 2 are not touched.

| slot pages | content |
|---|---|
| 0 | header (written **last**: a slot whose header checks out is complete) — run, the profile used, when each step began, thrust cal/tare |
| 1 – 128 | **the stand's own pages, byte for byte** (header, data, footer) — up to ~45 s of recording |
| 129 – 511 | the **merged table**, 15 rows per page, CRC32 per page |

Merged row (16 B): stand timestamp (µs from T0), the stand's raw 24-bit load-cell count with its flags
(pyro window, continuity, RBF) in the top byte plus the clamp phase (bits 28–30) and a *no Ariadna data
yet* bit (31), battery mV, current mA, servo µs, clamp closure in permille. Ariadna's own samples are
taken at 200 Hz; each stand sample carries the latest one at or before it. Time is aligned through the
stand's own clock (every STATUS says how long to / since T0), so the two sit on one axis to within
about a millisecond before T0 and a few hundred µs after it.

## Getting the data out

`p` on the console dumps every stored run: a **CSV block** of the merged table
(`t_s, thrust_N, vbat_V, current_A, servo_us, closure_pct, phase, pyro, cont, own_data`), then the
stand's pages in the **stand's own dump format**. The stand's tools talk to Ariadna's console unchanged:

```bat
cd Static_Fire_Stand\tools
python static_fire.py --port COM7            :: Ariadna's port: downloads, analyses the THRUST exactly as for a stand dump
cd ...\Examples\Static_Fire_Link
python extract_csv.py ..\..\..\..\..\Static_Fire_Stand\tools\results\<date>\raw_dump.txt
```

`l` lists the slots, `p <slot>` dumps one. `pull` copies the stand's last burn right now (after an
Ariadna reset, or a copy that was missed); `hold` / `go` say "not ready" / "ready" (and stop a countdown);
`wipe yes` erases all stored runs (about 15 s).

## What it does not do

* **Ariadna's own samples are in RAM during the burn.** A reset of the Master mid-burn loses them. The
  stand's record is power-fail safe on its own and is copied afterwards even without them (flagged
  *no own data*). The record is written to flash after the burn, not during it, because the burn is
  exactly when the PCA9685 bus and the flash have to stay out of the way.
* One run is kept in RAM: ~41 s at 200 Hz (`OWN_MAX`), more than the stand records.
* The stand's pages are kept up to 128 (~45 s at 80 SPS); a longer recording is flagged *partial*.
* No servo feedback (see above). The clamp moves as fast as the servo does; at 50 Hz PWM an analogue
  servo updates every 20 ms. Set `PWM_HZ` 339 for the digital servo of the flight clamp.
* The flight log and this example share the chip: a flight log that grows into the top 1 MB would
  erase the oldest runs. Erase them (`wipe yes`) before flashing the flight firmware back if you want
  the whole chip for the flight log.

## Tests

The decisions are in three headers with no hardware in them, compiled unchanged into the sketch:

| header | |
|---|---|
| `sf_flaps.h` | the clamp profile, its fail-safe, the percent ↔ µs mapping |
| `sf_run.h` | when to arm, when to fire, what to tell the stand — **and when not to** |
| `sf_record.h` | the flash layout, CRCs, the merge |
| `sf_link_proto.h` | the I2C frames; the **stand's repository has the canonical copy** |

```
cd test
make sflink        # profile, supervisor (a stale / foreign / late IGNITE, a dead link, ...), record + merge
                   # against real pages built by the stand's own Python tools
make sflink-loop   # both ends against each other, T0 read at every 100 us offset (needs the Static_Fire_Stand checkout)
make syntax        # the sketch against a do-nothing Arduino shim
make sflink-sync   # sf_link_proto.h still matches the stand's
```

`make sflink-golden STAND=../../Static_Fire_Stand` regenerates `sflink_golden.h`. None of this replaces
building with arduino-cli and a bench run with LEDs on the pyro output.
