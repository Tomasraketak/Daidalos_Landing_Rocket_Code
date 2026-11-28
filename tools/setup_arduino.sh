#!/usr/bin/env bash
# One-time (idempotent) setup of everything tools/arduino_build.sh needs, on
# Linux x86-64 (a cloud session, CI-like box, WSL):
#
#   tools/setup_arduino.sh            installs into ~/.arduino-ariadna
#   ARDUINO_HOME=/some/dir tools/setup_arduino.sh
#
# What it installs:
#   - arduino-cli (release tarball from GitHub, no install script)
#   - the arduino-pico core rp2040:rp2040 (Pico 2 / Pico 2 W / XIAO RP2350)
#   - Library Manager libraries: Adafruit NeoPixel, LoRa, Adafruit SSD1306,
#     Adafruit GFX Library, TinyGPSPlus, DW1000 (+ the return -1 patch the
#     README asks for: DW1000 0.9.0 does not build on arduino-pico as shipped)
#   - checkouts of Voltino_TriSense_Library, TriSenseINSLIB and
#     TF-Luna-I2C-Library (latest main; pulled again on every run)
# and writes $ARDUINO_HOME/env.sh, which tools/arduino_build.sh sources by
# itself - after this, just run tools/arduino_build.sh.
#
# Needs network access to github.com, downloads.arduino.cc and the GitHub
# release assets of earlephilhower/arduino-pico (~900 MB once, mostly the
# RP2350 toolchain).
set -eu

ARDUINO_HOME=${ARDUINO_HOME:-$HOME/.arduino-ariadna}
CLI_VERSION=${CLI_VERSION:-1.1.1}
mkdir -p "$ARDUINO_HOME/bin" "$ARDUINO_HOME/libs"
cd "$ARDUINO_HOME"

CLI="$ARDUINO_HOME/bin/arduino-cli"
if [ ! -x "$CLI" ]; then
    echo "== arduino-cli $CLI_VERSION"
    curl -fsSL -o cli.tgz \
        "https://github.com/arduino/arduino-cli/releases/download/v${CLI_VERSION}/arduino-cli_${CLI_VERSION}_Linux_64bit.tar.gz"
    tar -xzf cli.tgz -C bin arduino-cli
    rm -f cli.tgz
fi

CFG="$ARDUINO_HOME/cli.yaml"
cat > "$CFG" <<YAML
board_manager:
  additional_urls: [https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json]
directories:
  data: $ARDUINO_HOME/data
  downloads: $ARDUINO_HOME/dl
  user: $ARDUINO_HOME/user
YAML
A="$CLI --config-file $CFG"

echo "== arduino-pico core"
$A core update-index
$A core install rp2040:rp2040

echo "== Library Manager libraries"
$A lib update-index
$A lib install "Adafruit NeoPixel" "LoRa" "Adafruit SSD1306" "Adafruit GFX Library" "TinyGPSPlus" "DW1000"

DW="$ARDUINO_HOME/user/libraries/DW1000/src/DW1000Ranging.cpp"
python3 - "$DW" <<'PY'
import sys
p = sys.argv[1]; s = open(p).read()
a = "\t\treturn datas[SHORT_MAC_LEN];\n\t}\n}"
if a in s:
    open(p, "w").write(s.replace(a, "\t\treturn datas[SHORT_MAC_LEN];\n\t}\n\treturn -1;\n}"))
    print("DW1000: patched detectMessageType()")
PY

echo "== own libraries (latest main)"
for r in Voltino_TriSense_Library TriSenseINSLIB TF-Luna-I2C-Library; do
    if [ -d "libs/$r/.git" ]; then
        git -C "libs/$r" pull -q --ff-only || echo "warning: could not update $r"
    else
        git clone -q --depth 1 "https://github.com/Tomasraketak/$r" "libs/$r"
    fi
done

cat > "$ARDUINO_HOME/env.sh" <<ENV
# Written by tools/setup_arduino.sh - sourced by tools/arduino_build.sh.
: "\${ARDUINO_CLI:=$CLI}"
: "\${ARDUINO_CONFIG:=$CFG}"
: "\${TRISENSE:=$ARDUINO_HOME/libs/Voltino_TriSense_Library}"
: "\${INSLIB:=$ARDUINO_HOME/libs/TriSenseINSLIB}"
: "\${TFLUNA:=$ARDUINO_HOME/libs/TF-Luna-I2C-Library}"
export ARDUINO_CLI ARDUINO_CONFIG TRISENSE INSLIB TFLUNA
ENV

echo "== ready: $($CLI version)"
echo "   run tools/arduino_build.sh"
