#!/usr/bin/env bash
# Compile every sketch with the REAL Arduino toolchain (arduino-cli + the
# arduino-pico core), exactly as the Arduino IDE does - including the
# IDE's .ino preprocessing (generated function prototypes), which a plain
# g++ build (test/Makefile "make syntax") does not reproduce.
#
#   tools/arduino_build.sh                 all sketches
#   tools/arduino_build.sh Position_MCU    only sketches whose path contains the word
#
# Needs arduino-cli with the core rp2040:rp2040 (board manager URL
# https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json)
# and the libraries: Adafruit NeoPixel, LoRa, Adafruit SSD1306, Adafruit GFX
# Library, TinyGPSPlus, DW1000 (Library Manager) plus Voltino_TriSense,
# TriSenseINSLIB and TFLuna_I2C - installed, or as checkouts named by
#   TRISENSE=... INSLIB=... TFLUNA=...
# Optional: ARDUINO_CLI=<path to arduino-cli>, ARDUINO_CONFIG=<cli.yaml>.
# tools/setup_arduino.sh installs all of it and sets these up by itself.
# The CI workflow .github/workflows/arduino-build.yml runs this on every push.
set -u
cd "$(dirname "$0")/.."

# tools/setup_arduino.sh leaves its paths here; explicit variables still win.
ENV_SH="${ARDUINO_HOME:-$HOME/.arduino-ariadna}/env.sh"
[ -f "$ENV_SH" ] && . "$ENV_SH"

CLI=${ARDUINO_CLI:-arduino-cli}
[ -n "${ARDUINO_CONFIG:-}" ] && CLI="$CLI --config-file $ARDUINO_CONFIG"
LIBS=()
for d in "${TRISENSE:-}" "${INSLIB:-}" "${TFLUNA:-}"; do
    [ -n "$d" ] && LIBS+=(--library "$d")
done

PICO2=rp2040:rp2040:rpipico2
PICO2W=rp2040:rp2040:rpipico2w
XIAO=rp2040:rp2040:seeed_xiao_rp2350

# sketch directory              board    extra compiler flags
SKETCHES="
Daidalos_Landing_Guidance                 $PICO2  -
Ariadna_V2/Master_MCU                     $PICO2W -
Ariadna_V2/Position_MCU                   $XIAO   -
Ariadna_V2/Position_MCU                   $XIAO   -DARIADNA_USE_INSLIB=0
Ariadna_V2/Navigation_MCU                 $XIAO   -
Ariadna_V2/Ground_Station                 $PICO2  -
Ariadna_V2/Rampa_MCU                      $PICO2  -
Ariadna_V2/Uwb_Tag                        $XIAO   -
Ariadna_V2/Uwb_Anchor                     $XIAO   -
Ariadna_V2/Bench_Tests/Master_Test        $PICO2W -
Ariadna_V2/Bench_Tests/Navigation_Test    $XIAO   -
Ariadna_V2/Bench_Tests/Position_Test      $XIAO   -
Ariadna_V2/Examples/Actuator_Console      $PICO2W -
Ariadna_V2/Examples/Pyro_Companion        $XIAO   -
Ariadna_V2/Examples/Servo_RGB_Demo        $PICO2W -
Ariadna_V2/Examples/Static_Fire_Link      $PICO2W -
"

fail=0; n=0
while read -r dir fqbn flags; do
    [ -z "$dir" ] && continue
    [ $# -gt 0 ] && [[ "$dir" != *"$1"* ]] && continue
    n=$((n + 1))
    extra=()
    label="$dir"
    if [ "$flags" != "-" ]; then
        extra=(--build-property "compiler.cpp.extra_flags=$flags")
        label="$dir ($flags)"
    fi
    if out=$($CLI compile --fqbn "$fqbn" "${LIBS[@]}" "${extra[@]}" --warnings default "$dir" 2>&1); then
        echo "OK    $label  [$fqbn]  $(echo "$out" | grep -o 'Sketch uses [0-9]* bytes' | grep -o '[0-9]*') B flash"
    else
        echo "FAIL  $label  [$fqbn]"
        echo "$out" | grep -E "error|Error" | head -20 | sed 's/^/      /'
        fail=$((fail + 1))
    fi
done <<< "$SKETCHES"

echo "$((n - fail)) of $n sketches compiled"
[ $fail -eq 0 ]
