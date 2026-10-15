# Bench tests — Ariadna V2

Three standalone sketches, one per vehicle board, that check every chip, sensor,
servo and wire the flight firmware depends on, and print the result on the USB
serial console. They replace the flight firmware on the bench and are flashed back
over it afterwards.

| sketch | board | libraries |
|---|---|---|
| `Master_Test/Master_Test.ino` | Raspberry Pi Pico 2 W | LoRa (sandeepmistry), Adafruit_NeoPixel |
| `Position_Test/Position_Test.ino` | Seeed XIAO RP2350 | Adafruit_NeoPixel, arduino-dw1000\* |
| `Navigation_Test/Navigation_Test.ino` | Seeed XIAO RP2350 | Adafruit_NeoPixel |

\* Set `UWB_LIB` to 0 in `Position_Test.ino` if arduino-dw1000 is not installed: the
DWM1000 is still identified by its DEV_ID register, only ranging is skipped. The
registry version (0.9.0) does not build on arduino-pico as shipped — add `return -1;`
as the last line of `DW1000RangingClass::detectMessageType()` in `DW1000Ranging.cpp`.

Each sketch folder is self-contained (no `../` includes), so it opens and builds in
Arduino IDE or `arduino-cli` directly. Console: **115200 baud**, any terminal.

## Nothing blocks

A missing, dead or unplugged part is a `FAIL` line, never a hang:

- I²C has a 10 ms timeout; every sensor that fails is re-probed every 3–5 s, so it
  can be plugged in while the test runs.
- Flash erase/program and LoRa transmit are state machines polled from the loop.
- The inter-board link only writes a frame (≤ 32 B) into an **empty** UART FIFO.
- No `while (!Serial)`: a board runs the same with or without a laptop. The summary
  is reprinted every 5 s, so opening the terminal late loses nothing.
- Each summary shows `loop max … us`, the longest single pass of `loop()` — the
  proof that nothing stalled.

## What is tested

**Master** — PCA9685 (prescale written and read back), servos stepped one at a time
(TVC X, TVC Y: trim → +5° → trim → −5° → trim; clamp: mid → closed → mid → open),
W25Q128 JEDEC ID and log usage, erase/write/verify/erase of the **last** sector (the
flight log never reaches it, and it is left erased), LoRa init and a packet every 2 s
with TX-done taken from the DIO0 interrupt, battery voltage, ACS712 current, RBF and
breakwire (PASS once each has been seen in both states), RGB colour cycle, buzzer
jingle, GP22 square wave, core 1, both links, and the Navigation **reset line**: at
21 s it is pulsed and the Navigation MCU must be seen to reboot. `RESET_POS` is always
SKIP — GP6 is the PCA9685 /OE, held HIGH until the servos hold a neutral pulse.

The LoRa packet has the exact layout and version of the flight telemetry, with
`state = TEST`, so the ground station decodes it and the visualiser shows it — the
whole radio path is checked, not just the transmitter. `exp` carries a counter, so a
lost packet is visible. Transmit power is 10 dBm. **Never transmit without an antenna.**

**Position** — every sensor through its own registers rather than TriSense, so each
one passes or fails on its own: I²C idle levels and scan, ICM42688P (WHO_AM_I at
10 MHz, retried at 1 MHz to tell a wiring problem from a dead chip; |a| ≈ 1 g at rest,
frozen-data detection), BMP580 (CHIP_ID, pressure, temperature, noise in Pa and cm),
AK09918C (WIA, |B|, overflow), TF Luna (optional), LK76 GPS (auto-baud 9600 / 115200 /
38400 / 57600, checksummed NMEA, fix, satellites, HDOP), DWM1000 (DEV_ID, then ranging
to anchors 1–4), core 1, links to Master and to Navigation.

**Navigation** — pyro continuity on all three channels (raw ADC counts, present /
open, and a floating input flagged by its noise), pyro outputs held LOW and read back,
I²C scan of the unused D4/D5 pins, core 1, links to Master and to Position.

> **Pyro outputs.** The firing test is compiled out (`PYRO_OUTPUT_TEST 0`). With it
> set to 1, typing `y` drives CH1, CH2, CH3 HIGH for 200 ms each and reads the pin
> back. **LEDs only — never an igniter, never a motor.**

## Inter-board communication

Every board pings every other board at 10 Hz and answers pings addressed to it.
Position and Navigation are not wired to each other, so their pings to each other go
**through the Master**, which routes on the destination byte — the same path the
navigation state and the actuator commands take in flight. Reported per link: round
trip min/avg/max, loss, payload errors (a pattern derived from the sequence number,
checked even when the CRC passes), CRC errors, and dropped frames.

Position and Navigation also send their results to the Master once a second, so the
**Master's console shows all three boards** in one place.

Diagnoses the link test gives instead of just "dead":

| message | meaning |
|---|---|
| `no bytes at all on …` | wiring: TX↔RX swapped, no common GND, or the peer is off |
| `… runs FLIGHT firmware` | the peer is talking, but it is not running its bench test |
| `… protocol version mismatch` | the boards were flashed from different revisions |
| `… port receives packets from …` | the Master's two UARTs are crossed |

## Reading the output

```
[    3.42 s] PASS IMU_ID       ICM42688P WHO_AM_I 0x47 at 10 MHz
[    4.00 s] FAIL BARO_ID      no ACK at 0x47 or 0x46
---- POSITION self test @ 5.0 s ----
  PASS I2C_BUS      3 device(s) answer
  ...
  => pass 9  warn 1  fail 1  pending 2  skip 0  info 2
```

A line is printed whenever a verdict **changes**; the full table every 5 s.
`PASS` works, `WARN` works but read the detail, `FAIL` broken, `INFO` a reading rather
than a verdict (an open continuity channel on a bench without igniters), `SKIP`
disabled or untestable, `....` not decided yet.

The RGB LED: colour cycle red → green → blue → white at power-up (checks the LED
itself), then slow green = everything passes, slow amber = warnings or pending,
fast red = at least one FAIL.

## Console commands

| board | keys |
|---|---|
| Master | `h` help, `p` summary, `s` servo sweep on/off, `b` beep, `l` LoRa packet now, `f` rerun flash test, `i` I²C scan, `r` reset Position, `n` reset Navigation |
| Position | `h`, `p`, `i` I²C scan, `g` raw GPS sentences on/off |
| Navigation | `h`, `p`, `i` I²C scan, `y` pyro LED test (only if compiled in) |

## Keeping the copies in sync

`bench_common.h` (test registry, link framing, ping/pong, reports) is shared by all
three sketches. The original is `common/bench_common.h`; each sketch folder holds a
copy because Arduino only compiles what is inside the sketch folder.

```
cd ../../test
make bench            # host test of the bench link: framing, routing, ping/pong, reports
make bench-sync       # the three copies match the original
make bench-sync-fix   # refresh them after editing the original
```
