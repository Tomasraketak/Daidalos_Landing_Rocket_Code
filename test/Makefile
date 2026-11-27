# Host verification build. The flight algorithm compiles unchanged.
#
#   make          legacy V1 guidance (Daidalos_Landing_Guidance) against its model
#   make guid     guidance v2 (LandSim_Light port): golden vectors + closed loop
#   make seq      flight event sequencer and the parameter table
#   make est      Ariadna V2 navigation filter
#   make uwb      Ariadna V2 UWB multilateration
#   make uwblink  Ariadna V2 UWB tag I2C frame codec
#   make link     Ariadna V2 inter-processor link codec
#   make batt     Ariadna V2 battery state of charge and run-time estimate
#   make vib      Ariadna V2 vibration spectrum (tones, Parseval, overrun, 1 kHz, clock error)
#   make syntax   compile the sketches against a stub Arduino header
#   make sync     verify the sketches' copies of guidance and common/ headers
#   make pyfmt    verify ground_visualizer.py decodes the current packet
#   make bench    Ariadna V2 bench-test link (ping / pong / routing)
#   make bench-sync  verify the bench sketches' copies of bench_common.h
#   make actcon   Actuator Console pyro bridge: arming / firing safety rules
#   make sflink   Static Fire Link: when to fire, the clamp profile, the stored record
#   make sflink-sync  verify the stand's protocol header against the Static_Fire_Stand checkout
#   make sflink-loop  both ends of the Static Fire Link against each other (needs that checkout)
#   make calib    calibration fits of the diagnostic console (accel, magnetometer)
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
EX_DIR  = $(V2)/Examples
ACT_H   = $(V2)/common/actcon_pyro.h
SFL_DIR = $(EX_DIR)/Static_Fire_Link

all: host_test
	./host_test

host_test: $(SRC) ../Daidalos_Landing_Guidance/guidance.h golden_vectors.h
	$(CXX) $(CXXFLAGS) -o host_test $(SRC) -lm

# ---------------------------------------------------------------------
# Guidance v2. guid_golden.h and Ariadna_V2/common/guidance_tables.h are
# generated from the simulation: `make tables SIM=../../LandSim_Light`.
# ---------------------------------------------------------------------
GUID_SRC = $(V2)/common/guidance.cpp
GUID_DEP = $(V2)/common/guidance.h $(V2)/common/guidance_tables.h

guid: guid_test
	./guid_test

guid_test: guid_test.cpp $(GUID_SRC) $(GUID_DEP) guid_golden.h
	$(CXX) -O2 -Wall -Wextra -std=c++17 -o guid_test guid_test.cpp $(GUID_SRC) -lm
	$(CXX) $(CXXFLAGS) -fsyntax-only $(GUID_SRC)

seq: seq_test
	./seq_test

seq_test: seq_test.cpp $(V2)/common/ariadna_seq.h $(V2)/common/ariadna_params.h $(GUID_SRC) $(GUID_DEP)
	$(CXX) $(CXXFLAGS) -o seq_test seq_test.cpp $(GUID_SRC) -lm

SIM ?= ../../LandSim_Light
# Library checkouts, for the Position MCU syntax check (optional).
TRISENSE ?= ../../Voltino_TriSense_Library
INSLIB   ?= ../../TriSenseINSLIB
TFLUNA   ?= ../../TF-Luna-I2C-Library
# make tables SIM=../../LandSim_Light [VEHICLE=vehicle.json]  (VEHICLE in LandSim's format)
tables:
	python3 ../tools/gen_guidance_tables.py --sim $(SIM) $(if $(VEHICLE),--landsim-vehicle $(abspath $(VEHICLE)))

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
# Battery state of charge and run time to 20 % (fw 3.29). Header only, no
# Arduino dependency: this test and Master_MCU.ino compile the same file.
# ---------------------------------------------------------------------
BATT_H  = $(V2)/common/ariadna_batt.h

batt: batt_test
	./batt_test

batt_test: batt_test.cpp $(BATT_H)
	$(CXX) $(CXXFLAGS) -o batt_test batt_test.cpp -lm

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

RAMPA_H = $(V2)/common/ariadna_rampa_link.h

rampa: rampa_link_test
	./rampa_link_test

rampa_link_test: rampa_link_test.cpp $(RAMPA_H) $(LINK_H)
	$(CXX) $(CXXFLAGS) -o rampa_link_test rampa_link_test.cpp -lm

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
GUIDANCE_SRC = $(V2)/common

# original:copy pairs, relative to this directory
SYNC_PAIRS = \
  $(GUIDANCE_SRC)/guidance.h:$(NAV_DIR)/guidance.h \
  $(GUIDANCE_SRC)/guidance.cpp:$(NAV_DIR)/guidance.cpp \
  $(GUIDANCE_SRC)/guidance.h:$(V2)/Master_MCU/guidance.h \
  $(GUIDANCE_SRC)/guidance_tables.h:$(NAV_DIR)/guidance_tables.h \
  $(GUIDANCE_SRC)/guidance_tables.h:$(V2)/Master_MCU/guidance_tables.h \
  $(V2)/common/ariadna_params.h:$(NAV_DIR)/ariadna_params.h \
  $(V2)/common/ariadna_params_tbl.h:$(NAV_DIR)/ariadna_params_tbl.h \
  $(V2)/common/ariadna_params_tbl.h:$(V2)/Master_MCU/ariadna_params_tbl.h \
  $(V2)/common/ariadna_params_tbl.h:$(V2)/Position_MCU/ariadna_params_tbl.h \
  $(V2)/common/ariadna_params.h:$(V2)/Master_MCU/ariadna_params.h \
  $(V2)/common/ariadna_params.h:$(V2)/Position_MCU/ariadna_params.h \
  $(V2)/common/ariadna_seq.h:$(NAV_DIR)/ariadna_seq.h \
  $(V2)/common/ariadna_seq.h:$(V2)/Master_MCU/ariadna_seq.h \
  $(V2)/common/ariadna_seq.h:$(V2)/Position_MCU/ariadna_seq.h \
  $(V2)/common/actcon_pyro.h:$(NAV_DIR)/actcon_pyro.h \
  $(V2)/common/actcon_pyro.h:$(V2)/Master_MCU/actcon_pyro.h \
  $(V2)/common/ariadna_link.h:$(V2)/Master_MCU/ariadna_link.h \
  $(V2)/common/ariadna_rampa_link.h:$(V2)/Master_MCU/ariadna_rampa_link.h \
  $(V2)/common/ariadna_uart_dma.h:$(V2)/Master_MCU/ariadna_uart_dma.h \
  $(V2)/common/ariadna_batt.h:$(V2)/Master_MCU/ariadna_batt.h \
  $(V2)/common/ariadna_rgb.h:$(V2)/Master_MCU/ariadna_rgb.h \
  $(V2)/common/ariadna_rampa_link.h:$(V2)/Rampa_MCU/ariadna_rampa_link.h \
  $(V2)/common/ariadna_rgb.h:$(V2)/Rampa_MCU/ariadna_rgb.h \
  $(V2)/common/ariadna_link.h:$(V2)/Position_MCU/ariadna_link.h \
  $(V2)/common/ariadna_hil.h:$(V2)/Position_MCU/ariadna_hil.h \
  $(V2)/common/ariadna_uart_dma.h:$(V2)/Position_MCU/ariadna_uart_dma.h \
  $(V2)/common/ariadna_kf.h:$(V2)/Position_MCU/ariadna_kf.h \
  $(V2)/common/ariadna_vib.h:$(V2)/Position_MCU/ariadna_vib.h \
  $(V2)/common/ariadna_uwb.h:$(V2)/Position_MCU/ariadna_uwb.h \
  $(V2)/common/ariadna_uwb_link.h:$(V2)/Position_MCU/ariadna_uwb_link.h \
  $(V2)/common/ariadna_uwb_link.h:$(V2)/Uwb_Tag/ariadna_uwb_link.h \
  $(V2)/common/ariadna_rgb.h:$(V2)/Uwb_Tag/ariadna_rgb.h \
  $(V2)/common/ariadna_uwb_link.h:$(V2)/Bench_Tests/Position_Test/ariadna_uwb_link.h \
  $(V2)/common/ariadna_rgb.h:$(V2)/Position_MCU/ariadna_rgb.h \
  $(V2)/common/ariadna_link.h:$(NAV_DIR)/ariadna_link.h \
  $(V2)/common/ariadna_rampa_link.h:$(NAV_DIR)/ariadna_rampa_link.h \
  $(V2)/common/ariadna_uart_dma.h:$(NAV_DIR)/ariadna_uart_dma.h \
  $(V2)/common/ariadna_rgb.h:$(NAV_DIR)/ariadna_rgb.h \
  $(V2)/common/ariadna_link.h:$(V2)/Ground_Station/ariadna_link.h \
  $(V2)/common/ariadna_rgb.h:$(V2)/Uwb_Anchor/ariadna_rgb.h \
  $(V2)/common/ariadna_link.h:$(EX_DIR)/Actuator_Console/ariadna_link.h \
  $(V2)/common/actcon_pyro.h:$(EX_DIR)/Actuator_Console/actcon_pyro.h \
  $(V2)/common/ariadna_link.h:$(EX_DIR)/Pyro_Companion/ariadna_link.h \
  $(V2)/common/actcon_pyro.h:$(EX_DIR)/Pyro_Companion/actcon_pyro.h \
  $(V2)/common/ariadna_link.h:$(SFL_DIR)/ariadna_link.h \
  $(V2)/common/actcon_pyro.h:$(SFL_DIR)/actcon_pyro.h

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
# Hardware-in-the-Loop (HIL) USB injection protocol & 6-DOF simulator.
# ---------------------------------------------------------------------
HIL_H   = $(V2)/common/ariadna_hil.h

# ---------------------------------------------------------------------
# Vibration spectrum analyser (fw 3.35). Synthetic IMU streams with known
# tones and noise through the same tap the Position MCU uses.
# ---------------------------------------------------------------------
VIB_H = $(V2)/common/ariadna_vib.h

vib: vib_test
	./vib_test
	python3 -I check_vib_log.py

vib_test: vib_test.cpp $(VIB_H) $(LINK_H)
	$(CXX) -O2 -Wall -Wextra -std=c++17 -o vib_test vib_test.cpp -lm
	$(CXX) $(CXXFLAGS) -fsyntax-only -x c++ $(VIB_H)

hil: hil_test
	./hil_test

hil_test: hil_test.cpp $(HIL_H) $(LINK_H)
	$(CXX) $(CXXFLAGS) -o hil_test hil_test.cpp -lm

hilfmt:
	python3 check_hil_format.py

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
	$(CXX) $(CXXFLAGS) -DARDUINO=100 -Ifull_stub -I$(V2)/Master_MCU \
	    -x c++ -fsyntax-only $(V2)/Master_MCU/Master_MCU.ino
	$(CXX) $(CXXFLAGS) -DARDUINO=100 -Ifull_stub -I$(V2)/Rampa_MCU \
	    -x c++ -fsyntax-only $(V2)/Rampa_MCU/Rampa_MCU.ino
	@if [ -d "$(TRISENSE)/src" ] && [ -d "$(INSLIB)/src" ] && [ -d "$(TFLUNA)/src" ]; then \
	  for ins in 1 0; do \
	    $(CXX) -O2 -Wall -Wextra -std=c++17 -DARDUINO=100 -DARDUINO_ARCH_RP2350 -D__ARM_FP=4 \
	      -DARIADNA_USE_INSLIB=$$ins -Ifull_stub -I$(TRISENSE)/src -I$(INSLIB)/src -I$(TFLUNA)/src \
	      -I$(V2)/Position_MCU -x c++ -fsyntax-only $(V2)/Position_MCU/Position_MCU.ino || exit 1; \
	  done; echo "Position_MCU compiles against the TriSense, TriSenseINSLIB and TFLuna headers"; \
	else echo "Position_MCU: skipped (set TRISENSE=... INSLIB=... TFLUNA=... to the library checkouts)"; fi
	$(CXX) $(CXXFLAGS) -Iexample_stub -I$(EX_DIR)/Actuator_Console \
	    -x c++ -fsyntax-only $(EX_DIR)/Actuator_Console/Actuator_Console.ino
	$(CXX) $(CXXFLAGS) -Iexample_stub -I$(EX_DIR)/Pyro_Companion \
	    -x c++ -fsyntax-only $(EX_DIR)/Pyro_Companion/Pyro_Companion.ino
	$(CXX) $(CXXFLAGS) -Iexample_stub -I$(SFL_DIR) \
	    -x c++ -fsyntax-only $(SFL_DIR)/Static_Fire_Link.ino
	@echo "sketches compile clean"

# ---------------------------------------------------------------------
# Actuator Console (Examples/Actuator_Console + Examples/Pyro_Companion).
# An example, but one that can fire an igniter, so its arming, heartbeat
# and pulse rules are tested here like flight code.
# ---------------------------------------------------------------------
# Calibration fits of the diagnostic console (numpy; skipped without it).
calib:
	python3 -I check_calib.py

actcon: actcon_test
	./actcon_test

actcon_test: actcon_test.cpp $(ACT_H) $(LINK_H)
	$(CXX) $(CXXFLAGS) -o actcon_test actcon_test.cpp

# ---------------------------------------------------------------------
# Static Fire Link (Examples/Static_Fire_Link). It fires an igniter on a
# signal from another board and moves a clamp on a thrust threshold, so the
# rules for that are tested like flight code. The three headers under test
# are the sketch's own files, compiled unchanged.
#
# sflink_golden.h holds a real record in the stand's on-flash format, built
# and decoded by the stand's own Python tools:
#   make sflink-golden STAND=../../Static_Fire_Stand
# ---------------------------------------------------------------------
STAND ?= ../../Static_Fire_Stand
SFL_HDRS = $(SFL_DIR)/sf_flaps.h $(SFL_DIR)/sf_run.h $(SFL_DIR)/sf_record.h $(SFL_DIR)/sf_link_proto.h

sflink: sflink_test
	./sflink_test

sflink_test: sflink_test.cpp $(SFL_HDRS) sflink_golden.h
	$(CXX) $(CXXFLAGS) -o sflink_test sflink_test.cpp -lm

sflink-golden:
	python3 gen_sflink_golden.py --stand $(STAND) > sflink_golden.h

# Both ends of the link against each other (needs the stand checkout, since the
# stand's side of the logic lives there). Skipped, loudly, without it.
sflink-loop:
	@if [ -f $(STAND)/firmware/StaticFire_Stand/sf_stand_link.h ]; then \
	  $(CXX) $(CXXFLAGS) -DSTAND_LINK_H='"$(STAND)/firmware/StaticFire_Stand/sf_stand_link.h"' \
	    -o sflink_loop_test sflink_loop_test.cpp -lm && ./sflink_loop_test; \
	else echo "sflink-loop: skipped (no Static_Fire_Stand checkout at $(STAND))"; fi

# The protocol header exists twice (the Arduino builder only sees a sketch's own
# folder): the canonical one in the stand's repository, a copy here.
sflink-sync:
	@if [ -f $(STAND)/firmware/StaticFire_Stand/sf_link_proto.h ]; then \
	  cmp -s $(STAND)/firmware/StaticFire_Stand/sf_link_proto.h $(SFL_DIR)/sf_link_proto.h \
	    || { echo "sf_link_proto.h has drifted from $(STAND) - copy it over (the stand's is canonical)"; exit 1; }; \
	  echo "sf_link_proto.h matches the Static_Fire_Stand checkout"; \
	else echo "sflink-sync: skipped (no Static_Fire_Stand checkout at $(STAND))"; fi

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

check: all guid seq est uwb uwblink rampa link hil batt vib sync pyfmt hilfmt calib syntax bench bench-sync actcon sflink sflink-sync sflink-loop

# Regenerate the golden vectors from the simulation. Needs the model repo.
MODEL ?= ../../Landing-Rocket-Sim/lander_v2.py
golden:
	python3 gen_golden.py --model $(MODEL) > golden_vectors.h

clean:
	rm -f host_test guid_test seq_test link_test hil_test est_test uwb_test uwb_link_test rampa_link_test actcon_test batt_test vib_test bench_test sflink_test sflink_loop_test
.PHONY: all guid seq tables est uwb uwblink rampa link hil batt vib sync sync-fix pyfmt hilfmt syntax check golden clean actcon bench bench-sync bench-sync-fix sflink sflink-golden sflink-sync sflink-loop
