# Library patches

## `voltino_trisense_sample_tap.patch` — Voltino_TriSense 1.6.0

Adds `ICM42688P::setSampleTap(fn, ctx)`: a callback with every IMU sample the
driver hands out through `readIMU()` / `readFIFO()`, in the same calibrated
units (g, deg/s). TriSenseINSLIB owns the IMU FIFO; reading it a second time
would steal samples from the attitude filter (the fw 3.21 bug). The tap lets
the Position MCU's vibration analyser (fw 3.35, `common/ariadna_vib.h`,
USER_MANUAL.md chapter 17) see the full 2 kHz stream without touching the bus.
Cost when nothing is installed: one pointer test per sample.

```
cd Voltino_TriSense_Library
git apply /path/to/Daidalos_Landing_Rocket_Code/Ariadna_V2/patches/voltino_trisense_sample_tap.patch
```

The firmware detects the method at compile time (`vib_install_tap()`): without
the patch it still builds and flies, the analyser just stays off and the
Position MCU says so on its console at boot. Once the change is merged into
the library itself, this patch is no longer needed.
