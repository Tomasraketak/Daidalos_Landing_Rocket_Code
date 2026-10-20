# Host verification build. The flight algorithm compiles unchanged.
#
#   make          numerical check of the algorithm against the Python model
#   make est      Ariadna V2 navigation filter
#   make uwb      Ariadna V2 UWB multilateration
#   make uwblink  Ariadna V2 UWB tag I2C frame codec
#   make link     Ariadna V2 inter-processor link codec
#   make syntax   compile the sketches against a stub Arduino header
#   make sync     verify the sketches' copies of guidance and common/ headers
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
UWBL_H  = $(V2)/common/ariadna_uwb_link.h

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
# this test and unchanged into Position_MCU.ino, so the arithmetic that
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
# Position_MCU.ino.
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
# The UWB tag's I2C frame. The tag board and the Position MCU are built
# separately and flashed separately, so this header is the only thing that
# keeps them agreeing; a bus that answers 0xFF, a short read and a flipped
# bit are all checked here, where they are easy to produce.
# ---------------------------------------------------------------------
uwblink: uwb_link_test
	./uwb_link_test

uwb_link_test: uwb_link_test.cpp $(UWBL_H)
	$(CXX) $(CXXFLAGS) -o uwb_link_test uwb_link_test.cpp -lm

# ---------------------------------------------------------------------
# The Arduino builder compiles a sketch from a copy of its own folder, so
# nothing outside that folder can be reached — not a .cpp, and not a
# "../common/x.h" include either. Every sketch therefore carries its own
# copy of the guidance module and of the common/ headers it uses.
#
# A copy that silently drifts from the verified original is exactly the
# failure this repository exists to prevent, so the copies are checked
# rather than trusted. `make sync-fix` refreshes them all.
# ---------------------------------------------------------------------
GUIDANCE_SRC = ../Daidalos_Landing_Guidance

# original:copy pairs, relative to this directory
SYNC_PAIRS = \
  $(GUIDANCE_SRC)/guidance.h:$(NAV_DIR)/guidance.h \
  $(GUIDANCE_SRC)/guidance.cpp:$(NAV_DIR)/guidance.cpp \
  $(GUIDANCE_SRC)/guidance.h:$(V2)/Master_MCU/guidance.h \
  $(V2)/common/ariadna_link.h:$(V2)/Master_MCU/ariadna_link.h \
  $(V2)/common/ariadna_uart_dma.h:$(V2)/Master_MCU/ariadna_uart_dma.h \
  $(V2)/common/ariadna_rgb.h:$(V2)/Master_MCU/ariadna_rgb.h \
  $(V2)/common/ariadna_link.h:$(V2)/Position_MCU/ariadna_link.h \
  $(V2)/common/ariadna_uart_dma.h:$(V2)/Position_MCU/ariadna_uart_dma.h \
  $(V2)/common/ariadna_kf.h:$(V2)/Position_MCU/ariadna_kf.h \
  $(V2)/common/ariadna_uwb.h:$(V2)/Position_MCU/ariadna_uwb.h \
  $(V2)/common/ariadna_uwb_link.h:$(V2)/Position_MCU/ariadna_uwb_link.h \
  $(V2)/common/ariadna_uwb_link.h:$(V2)/Uwb_Tag/ariadna_uwb_link.h \
  $(V2)/common/ariadna_rgb.h:$(V2)/Uwb_Tag/ariadna_rgb.h \
  $(V2)/common/ariadna_uwb_link.h:$(V2)/Bench_Tests/Position_Test/ariadna_uwb_link.h \
  $(V2)/common/ariadna_rgb.h:$(V2)/Position_MCU/ariadna_rgb.h \
  $(V2)/common/ariadna_link.h:$(NAV_DIR)/ariadna_link.h \
  $(V2)/common/ariadna_uart_dma.h:$(NAV_DIR)/ariadna_uart_dma.h \
  $(V2)/common/ariadna_rgb.h:$(NAV_DIR)/ariadna_rgb.h \
  $(V2)/common/ariadna_link.h:$(V2)/Ground_Station/ariadna_link.h \
  $(V2)/common/ariadna_rgb.h:$(V2)/Uwb_Anchor/ariadna_rgb.h

sync:
	@fail=0; for p in $(SYNC_PAIRS); do \
	  src=$${p%%:*}; dst=$${p#*:}; \
	  cmp -s $$src $$dst || { echo "$$dst has drifted from $$src"; fail=1; }; \
	done; \
	if [ $$fail -ne 0 ]; then echo "run 'make sync-fix'"; exit 1; fi
	@echo "every sketch's copy of guidance and common/ matches the original"

sync-fix:
	@for p in $(SYNC_PAIRS); do cp $${p%%:*} $${p#*:}; done
	@echo "guidance module and common/ headers re-synced into the sketches"

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
# checked. Master_MCU.ino, Position_MCU.ino and Ground_Station.ino pull in
# LoRa, Adafruit_SSD1306 and TriSense; stubbing those faithfully would be
# more code than the sketches, and a stub that drifts from the real library
# gives false confidence rather than none. Build those in Arduino IDE.
# ---------------------------------------------------------------------
syntax: $(INO) stub/Arduino.h
	$(CXX) $(CXXFLAGS) -Istub -I../Daidalos_Landing_Guidance \
	    -x c++ -fsyntax-only $(INO)
	$(CXX) $(CXXFLAGS) -Istub -I$(NAV_DIR) \
	    -x c++ -fsyntax-only $(NAV_DIR)/Navigation_MCU.ino
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

check: all est uwb uwblink link sync pyfmt syntax bench bench-sync

# Regenerate the golden vectors from the simulation. Needs the model repo.
MODEL ?= ../../Landing-Rocket-Sim/lander_v2.py
golden:
	python3 gen_golden.py --model $(MODEL) > golden_vectors.h

clean:
	rm -f host_test link_test est_test uwb_test uwb_link_test
.PHONY: all est uwb uwblink link sync sync-fix pyfmt syntax check golden clean
