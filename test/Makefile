# Host verification build. The flight algorithm compiles unchanged.
#
#   make          numerical check of the algorithm against the Python model
#   make link     Ariadna V2 inter-processor link codec
#   make syntax   compile the sketches against a stub Arduino header
#   make sync     verify the Navigation MCU's guidance copy has not drifted
#   make pyfmt    verify ground_visualizer.py decodes the current packet
#   make check    all of the above
#
# -Wdouble-promotion is not cosmetic here: the RP2350 has a single-precision
# FPU only, so a promoted expression is emulated in software and 10-50x
# slower. A warning from this build is a real performance bug on the target.
CXX      ?= g++
CXXFLAGS ?= -O2 -Wall -Wextra -Wdouble-promotion -std=c++17
SRC = host_test.cpp ../Daidalos_Landing_Guidance/guidance.cpp
INO = ../Daidalos_Landing_Guidance/Daidalos_Landing_Guidance.ino

V2      = ../Ariadna_V2
NAV_DIR = $(V2)/Navigation_MCU
LINK_H  = $(V2)/common/ariadna_link.h

all: host_test
	./host_test

host_test: $(SRC) ../Daidalos_Landing_Guidance/guidance.h golden_vectors.h
	$(CXX) $(CXXFLAGS) -o host_test $(SRC) -lm

# ---------------------------------------------------------------------
# Ariadna V2 link codec. Shared by three MCUs, a ground station and a
# Python client, so a framing bug is a four-way problem. Tested here
# rather than on the pad.
# ---------------------------------------------------------------------
link: link_test
	./link_test

link_test: link_test.cpp $(LINK_H)
	$(CXX) $(CXXFLAGS) -o link_test link_test.cpp

# ---------------------------------------------------------------------
# Arduino cannot compile a .cpp that lives outside the sketch folder, so
# the Navigation MCU sketch carries its own copy of the guidance module.
# A copy that silently drifts from the verified original is exactly the
# failure this repository exists to prevent, so the copy is checked rather
# than trusted. `make sync-fix` refreshes it.
# ---------------------------------------------------------------------
sync:
	@cmp -s ../Daidalos_Landing_Guidance/guidance.h   $(NAV_DIR)/guidance.h \
	  || { echo "guidance.h in $(NAV_DIR) has drifted - run 'make sync-fix'"; exit 1; }
	@cmp -s ../Daidalos_Landing_Guidance/guidance.cpp $(NAV_DIR)/guidance.cpp \
	  || { echo "guidance.cpp in $(NAV_DIR) has drifted - run 'make sync-fix'"; exit 1; }
	@echo "Navigation MCU guidance copy matches the verified original"

sync-fix:
	cp ../Daidalos_Landing_Guidance/guidance.h   $(NAV_DIR)/guidance.h
	cp ../Daidalos_Landing_Guidance/guidance.cpp $(NAV_DIR)/guidance.cpp
	@echo "guidance module re-synced into $(NAV_DIR)"

# ---------------------------------------------------------------------
# The ground station's Python client decodes the downlink with a struct
# format string. link_test.cpp pins the C layout; this pins the Python one
# against it, so the two cannot drift apart unnoticed.
# ---------------------------------------------------------------------
pyfmt:
	python3 check_py_format.py

# ---------------------------------------------------------------------
# Type- and format-check the sketches without a board. Compile only:
# setup() and loop() are entry points for the Arduino core, so there is no
# main().
#
# Only the two sketches whose dependencies are the Arduino core itself are
# checked. master_mcu.ino, position_mcu.ino and ground_station.ino pull in
# LoRa, Adafruit_SSD1306 and TriSense; stubbing those faithfully would be
# more code than the sketches, and a stub that drifts from the real library
# gives false confidence rather than none. Build those in Arduino IDE.
# ---------------------------------------------------------------------
syntax: $(INO) stub/Arduino.h
	$(CXX) $(CXXFLAGS) -Istub -I../Daidalos_Landing_Guidance \
	    -x c++ -fsyntax-only $(INO)
	$(CXX) $(CXXFLAGS) -Istub -I$(NAV_DIR) \
	    -x c++ -fsyntax-only $(NAV_DIR)/navigation_mcu.ino
	@echo "sketches compile clean"

check: all link sync pyfmt syntax

# Regenerate the golden vectors from the simulation. Needs the model repo.
MODEL ?= ../../Landing-Rocket-Sim/lander_v2.py
golden:
	python3 gen_golden.py --model $(MODEL) > golden_vectors.h

clean:
	rm -f host_test link_test
.PHONY: all link sync sync-fix pyfmt syntax check golden clean
