# Host verification build. The flight algorithm compiles unchanged.
#
#   make          numerical check of the algorithm against the Python model
#   make syntax   compile the .ino itself against a stub Arduino header
#   make check    both
#
# -Wdouble-promotion is not cosmetic here: the RP2350 has a single-precision
# FPU only, so a promoted expression is emulated in software and 10-50x
# slower. A warning from this build is a real performance bug on the target.
CXX      ?= g++
CXXFLAGS ?= -O2 -Wall -Wextra -Wdouble-promotion -std=c++17
SRC = host_test.cpp ../Daidalos_Landing_Guidance/guidance.cpp
INO = ../Daidalos_Landing_Guidance/Daidalos_Landing_Guidance.ino

all: host_test
	./host_test

host_test: $(SRC) ../Daidalos_Landing_Guidance/guidance.h golden_vectors.h
	$(CXX) $(CXXFLAGS) -o host_test $(SRC) -lm

# Type- and format-check the sketch without a board. Compile only: setup()
# and loop() are entry points for the Arduino core, so there is no main().
syntax: $(INO) stub/Arduino.h
	$(CXX) $(CXXFLAGS) -Istub -I../Daidalos_Landing_Guidance \
	    -x c++ -fsyntax-only $(INO)
	@echo "sketch compiles clean"

check: all syntax

# Regenerate the golden vectors from the simulation. Needs the model repo.
MODEL ?= ../../Landing-Rocket-Sim/lander_v2.py
golden:
	python3 gen_golden.py --model $(MODEL) > golden_vectors.h

clean:
	rm -f host_test
.PHONY: all syntax check golden clean
