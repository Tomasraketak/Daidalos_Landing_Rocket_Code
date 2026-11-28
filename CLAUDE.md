# Daidalos / Ariadna V2 — notes for Claude

Flight software for the Daidalos landing rocket: Arduino sketches for RP2350 boards
(arduino-pico core), shared headers in `Ariadna_V2/common/`, host tests in `test/`,
Python ground tools in `Ariadna_V2/Diagnostics/` and `tools/`. The owner builds and
flashes in the **Arduino IDE on Windows** — a sketch that does not compile there is a
broken deliverable, whatever the host tests say.

## Before handing over ANY change to a sketch or a header: compile for real

```bash
tools/setup_arduino.sh        # once per machine/session, ~1 min, idempotent
tools/arduino_build.sh        # all 16 sketch builds, must end "16 of 16 sketches compiled"
tools/arduino_build.sh Position_MCU   # just the sketches whose path contains the word
cd test && make check         # host tests + header-copy sync
```

`setup_arduino.sh` installs arduino-cli (release tarball, no install script), the
arduino-pico core `rp2040:rp2040`, the Library Manager libraries, the DW1000 patch and
checkouts of the owner's own libraries into `~/.arduino-ariadna`; `arduino_build.sh` picks
that up by itself. Do not report a sketch change as done on `make syntax` alone: it uses
plain g++ and misses the IDE-only failures below. CI (`.github/workflows/arduino-build.yml`)
runs both on every push — check it went green before merging.

## Arduino IDE pitfalls that have broken the build here

The IDE writes a prototype for every function defined in a `.ino` and puts all of them
above the FIRST function in the file.

- A function whose signature uses a struct defined further down fails
  (`'LoRaSyncSnap' was not declared in this scope`). Forward-declare the struct near the
  top (`struct LoRaSyncSnap;`), as Master_MCU does.
- **Templates** (and `auto f(...) -> decltype(...)`) in a `.ino` cannot be prototyped. Put
  them in a `.h` next to the sketch (`Position_MCU/ariadna_imu_compat.h`).
- Do not add a function definition near the top of a `.ino`: it becomes the "first
  function" and drags every prototype above the structs they need (`'GpsFix' does not
  name a type`).
- Never name a constant like an arduino-pico macro: `PIN_LED`, `PIN_SPI0_*`,
  `PIN_SERIAL*`, `LED_BUILTIN`, `SS`, `MOSI`, … (they are `#define`d in `pins_arduino.h`).

## Boards (FQBN)

| Sketch | Board |
|---|---|
| Master_MCU, Bench_Tests/Master_Test, Examples Actuator_Console / Servo_RGB_Demo / Static_Fire_Link | Pico 2 W `rp2040:rp2040:rpipico2w` |
| Position_MCU, Navigation_MCU, Uwb_Tag, Uwb_Anchor, Bench_Tests Navigation/Position, Examples/Pyro_Companion | Seeed XIAO RP2350 `rp2040:rp2040:seeed_xiao_rp2350` |
| Ground_Station, Rampa_MCU, Daidalos_Landing_Guidance | Pico 2 `rp2040:rp2040:rpipico2` |

## The owner's libraries (separate repositories)

- `Tomasraketak/Voltino_TriSense_Library` — IMU/baro/mag driver. Position_MCU needs
  **>= 1.7.1** (2 kHz IMU; 1.7.0 calibrates the gyro to ~-150 deg/s). Host tests:
  `make -C tests/host test`.
- `Tomasraketak/TriSenseINSLIB` — the INS Position_MCU uses (`TriSenseINS.h`).
- `Tomasraketak/TF-Luna-I2C-Library` — lidar, >= 1.1.0.

To change one of them, attach it with `add_repo` (access "push"); for a build check the
checkouts made by `setup_arduino.sh` are enough. A library change that a sketch depends on
goes in first, then the sketch.

## Other conventions

- `Ariadna_V2/common/*.h` are copied into each sketch folder; `make sync` (part of
  `make check`) fails if a copy drifted — edit `common/`, then copy, and add new copies to
  the sync list in `test/Makefile`.
- `ARIADNA_FW_VERSION` lives in every copy of `ariadna_link.h`; bumping it means every board
  is reflashed (the GUI compares the versions). Do not bump it for a single-board fix.
- The owner writes in Czech; answer in Czech.
