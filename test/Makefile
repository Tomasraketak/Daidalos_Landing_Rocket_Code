# Host verification build. The flight algorithm compiles unchanged.
#
#   make          numerical check of the algorithm against the Python model
#   make est      Ariadna V2 navigation filter
#   make uwb      Ariadna V2 UWB multilateration
#   make link     Ariadna V2 inter-processor link codec
#   make syntax   compile the sketches against a stub Arduino header
#   make sync     verify the Navigation MCU's guidance copy has not drifted
#   make pyfmt    verify ground_visualizer.py decodes the current packet
#   make bench    Ariadna V2 bench-test link (ping / pong / routing)
#   make bench-sync  verify the bench sketches' copies of bench_common.h
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
KF_H    = $(V2)/common/ariadna_kf.h
UWB_H   = $(V2)/common/ariadna_uwb.h

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
# Ariadna V2 navigation filter. ariadna_kf.h compiles unchanged into
# this test and unchanged into position_mcu.ino, so the arithmetic that
# is checked here is the arithmetic that flies. The sketch itself cannot
# be host-compiled — it pulls in TriSense — which is precisely why the
# filter was moved out of it.
# ---------------------------------------------------------------------
est: est_test
	./est_test

est_test: est_test.cpp $(KF_H)
	$(CXX) $(CXXFLAGS) -o est_test est_test.cpp -lm

# ---------------------------------------------------------------------
# Ariadna V2 UWB multilateration. Same discipline as the filter above:
# ariadna_uwb.h compiles unchanged into this test and unchanged into
# position_mcu.ino.
#
# This one earns its place more than most. The solver's whole job is to
# behave sensibly when the constellation is INCOMPLETE, and a partial
# constellation is difficult to arrange deliberately on a launch site and
# trivial to arrange here. Anchors are removed one at a time, fed NaNs,
# given negative ranges, placed on top of each other and put in a line.
# ---------------------------------------------------------------------
uwb: uwb_test
	./uwb_test

uwb_test: uwb_test.cpp $(UWB_H)
	$(CXX) $(CXXFLAGS) -o uwb_test uwb_test.cpp -lm

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

# ---------------------------------------------------------------------
# Ariadna V2 bench tests. The sketches in Bench_Tests/ exist to tell a
# broken wire from a broken board, so the link they use to do it is
# checked here. bench_common.h is copied into each sketch folder for the
# same reason the guidance module is copied above; bench-sync checks the
# copies, bench-sync-fix refreshes them.
# ---------------------------------------------------------------------
BENCH_DIR = $(V2)/Bench_Tests
BENCH_H   = $(BENCH_DIR)/common/bench_common.h
BENCH_SKETCHES = Master_Test Position_Test Navigation_Test

bench: bench_test
	./bench_test

bench_test: bench_test.cpp $(BENCH_H) bench_stub/Arduino.h bench_stub/hardware/uart.h
	$(CXX) $(CXXFLAGS) -Ibench_stub -o bench_test bench_test.cpp

bench-sync:
	@for s in $(BENCH_SKETCHES); do \
	  cmp -s $(BENCH_H) $(BENCH_DIR)/$$s/bench_common.h \
	    || { echo "$$s/bench_common.h has drifted - run 'make bench-sync-fix'"; exit 1; }; \
	done
	@echo "bench sketches' bench_common.h copies match the original"

bench-sync-fix:
	@for s in $(BENCH_SKETCHES); do cp $(BENCH_H) $(BENCH_DIR)/$$s/bench_common.h; done
	@echo "bench_common.h re-synced into $(BENCH_SKETCHES)"

check: all est uwb link sync pyfmt syntax bench bench-sync

# Regenerate the golden vectors from the simulation. Needs the model repo.
MODEL ?= ../../Landing-Rocket-Sim/lander_v2.py
golden:
	python3 gen_golden.py --model $(MODEL) > golden_vectors.h

clean:
	rm -f host_test link_test est_test uwb_test
.PHONY: all est uwb link sync sync-fix pyfmt syntax check golden clean
